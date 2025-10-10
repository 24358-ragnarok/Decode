package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.software.Drivetrain;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

import java.lang.reflect.Method;
import java.util.List;

/**
 * Test OpMode to verify the angle-based aiming calculations.
 * This version uses Java Reflection to call private methods within the
 * TrajectoryEngine,
 * allowing for a deep debug of its internal calculations without modifying the
 * engine itself.
 */
@TeleOp(name = "Test: Deep Aiming Debug", group = "Tests")
public class AimingCalculationTest extends LinearOpMode {
	
	@Override
	public void runOpMode() {
		UnifiedLogging logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		logging.enableRetainedMode();
		
		// --- HARDWARE & ENGINE INITIALIZATION ---
		Servo pitchServo = hardwareMap.get(Servo.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
		MatchSettings matchSettings = new MatchSettings(blackboard);
		matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE);
		matchSettings.setAutoStartingPosition(MatchSettings.AutoStartingPosition.FAR);
		
		
		// Create a Drivetrain instance required by the engine
		Drivetrain drivetrain = new Drivetrain(hardwareMap, matchSettings);
		drivetrain.follower.setStartingPose(Settings.Field.RESET_POSE);
		
		
		TrajectoryEngine trajectoryEngine = new TrajectoryEngine(null, matchSettings, drivetrain);
		
		// --- REFLECTION SETUP ---
		// Get handles to the private methods we want to inspect
		Method getDistanceFromGoalMethod = null;
		Method initialBallSpeedFromRPMMethod = null;
		try {
			getDistanceFromGoalMethod = TrajectoryEngine.class.getDeclaredMethod("getDistanceFromGoal", Pose.class,
					Pose.class);
			getDistanceFromGoalMethod.setAccessible(true);
			
			initialBallSpeedFromRPMMethod = TrajectoryEngine.class.getDeclaredMethod("initialBallSpeedFromRPM",
					double.class);
			initialBallSpeedFromRPMMethod.setAccessible(true);
			
		} catch (NoSuchMethodException e) {
			logging.addLine("Reflection setup failed: " + e.getMessage());
			logging.update();
			sleep(5000);
			return;
		}
		
		logging.addLine("Deep Aiming Debug Ready");
		logging.addLine("Shows internal TrajectoryEngine calculations");
		logging.update();
		
		waitForStart();
		
		while (opModeIsActive()) {
			drivetrain.update();
			// --- 1. GET CURRENT STATE ---
			double currentPitchDegrees = Settings.Launcher.servoToPitch(pitchServo.getPosition());
			Pose robotPose = drivetrain.getPose();
			Pose goalPose = Settings.Field.BLUE_GOAL_POSE;
			
			logging.addLine("=== 1. CURRENT STATE & INPUTS ===");
			logging.addData("Robot Pose", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
			logging.addData("Current Pitch", "%.2f°", currentPitchDegrees);
			
			// --- 2. CALCULATE INITIAL PARAMETERS (using reflection) ---
			try {
				// Call the private getDistanceFromGoal method
				double radiusFromGoal = (double) getDistanceFromGoalMethod.invoke(trajectoryEngine, robotPose,
						goalPose);
				
				// Calculate dynamic launcher height and relative height for the solver
				double currentLauncherHeight = TrajectoryEngine.calculateLauncherHeight(currentPitchDegrees);
				double targetHeight = Settings.Field.BLUE_GOAL_AIM_3D[2];
				double relativeHeight_h = targetHeight - currentLauncherHeight;
				
				logging.addLine("---");
				logging.addLine("=== 2. PRE-SOLVER CALCULATIONS ===");
				logging.addData("Distance to Goal (r)", "%.2f in", radiusFromGoal);
				logging.addData("Launcher Height", "%.2f in", currentLauncherHeight);
				logging.addData("Target Height", "%.2f in", targetHeight);
				logging.addData("Solver Relative Height (h)", "%.2f in", relativeHeight_h);
				
				// --- 3. REPLICATE THE SOLVER LOOP (using reflection) ---
				logging.addLine("---");
				logging.addLine("=== 3. SOLVER LOOP ANALYSIS ===");
				
				double bestAngle = Double.NaN;
				double bestRPM = Double.NaN;
				double maxRatio = Double.NEGATIVE_INFINITY;
				
				// Iterate through the same RPM range as the actual engine
				for (double rpm = Settings.Aiming.MIN_WHEEL_SPEED_RPM; rpm <= Settings.Aiming.MAX_WHEEL_SPEED_RPM; rpm += 500) { // Larger step for readability
					
					// Call the private initialBallSpeedFromRPM method
					double initialBallSpeed = (double) initialBallSpeedFromRPMMethod.invoke(trajectoryEngine, rpm);
					
					logging.addLine(String.format("--- RPM: %.0f ---", rpm));
					logging.addData("  -> Initial Speed (V0)", "%.2f in/s", initialBallSpeed);
					
					// Call the public solveForTheta method
					List<Double> launchAngles = trajectoryEngine.solveForTheta(initialBallSpeed, radiusFromGoal,
							relativeHeight_h);
					
					if (launchAngles.isEmpty()) {
						logging.addLine("  -> No solution at this RPM");
						continue;
					}
					
					for (Double theta : launchAngles) {
						double finalVelocityX = initialBallSpeed * Math.cos(theta);
						double t = radiusFromGoal / finalVelocityX;
						double finalVelocityY = initialBallSpeed * Math.sin(theta)
								- Settings.Aiming.GRAVITY_INCHES_PER_SEC_SQ * t;
						double currentRatio = Math.abs(finalVelocityY / finalVelocityX);
						
						logging.addData(String.format("  -> Angle: %.2f°", Math.toDegrees(theta)),
								String.format("Ratio: %.3f", currentRatio));
						
						if (currentRatio > maxRatio) {
							maxRatio = currentRatio;
							bestAngle = Math.toDegrees(theta);
							bestRPM = rpm;
						}
					}
				}
				
				logging.addLine("---");
				logging.addLine("=== 4. BEST SOLUTION FOUND (DEBUG) ===");
				if (!Double.isNaN(bestRPM)) {
					logging.addData("Best RPM", "%.0f", bestRPM);
					logging.addData("Best Launch Angle", "%.2f°", bestAngle);
					logging.addData("Max Ratio", "%.3f", maxRatio);
				} else {
					logging.addLine("No valid solution found in loop.");
				}
				
				// --- 5. GET FINAL RESULT FROM ACTUAL ENGINE (for comparison) ---
				TrajectoryEngine.AimingSolution solution = trajectoryEngine.getAimingOffsets(
						matchSettings.getAllianceColor(), currentPitchDegrees);
				
				logging.addLine("---");
				logging.addLine("=== 5. FINAL ENGINE RESULT ===");
				if (solution.hasTarget) {
					logging.addData("Required Launch Angle", "%.2f°", solution.verticalOffsetDegrees);
					logging.addData("Required RPM", "%.0f", solution.getRequiredWheelSpeedRPM());
					logging.addData("Pitch Change Needed", "%.2f°",
							solution.verticalOffsetDegrees - currentPitchDegrees);
				} else {
					logging.addLine("Engine reports NO valid solution.");
				}
				
			} catch (Exception e) {
				logging.addLine("Error during reflection: " + e.getMessage());
			}
			
			logging.update();
			sleep(2000); // Slower update rate to allow for reading the data
		}
	}
}