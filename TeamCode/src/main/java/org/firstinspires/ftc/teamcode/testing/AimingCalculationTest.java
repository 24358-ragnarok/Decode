package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.software.LimelightManager;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

import java.util.List;

/**
 * Test OpMode to verify the angle-based aiming calculations.
 * Shows raw Limelight data and the calculated trajectory values using the
 * actual TrajectoryEngine.
 */
@TeleOp(name = "Test: Aiming Calculations", group = "Tests")
public class AimingCalculationTest extends LinearOpMode {
	
	// Define the specific AprilTag ID we are looking for.
	private static final int TARGET_TAG_ID = 20; // Blue alliance basket
	
	// Declare hardware objects
	private Limelight3A limelight;
	private Servo pitchServo;
	private TrajectoryEngine trajectoryEngine;
	private MatchSettings matchSettings;
	
	@Override
	public void runOpMode() {
		UnifiedLogging logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		logging.enableRetainedMode();
		logging.addLine("Aiming Calculation Test Ready");
		logging.addLine("Point camera at AprilTag " + TARGET_TAG_ID);
		logging.addLine("Uses actual TrajectoryEngine + real servo readings");
		logging.update();
		
		// Initialize hardware
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(11);
		limelight.pipelineSwitch(1);
		
		pitchServo = hardwareMap.get(Servo.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
		
		// Initialize trajectory engine
		matchSettings = new MatchSettings(blackboard);
		matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE); // Testing with blue alliance
		LimelightManager limelightManager = new LimelightManager(limelight, matchSettings);
		
		// Create minimal drivetrain for trajectory engine (test only)
		org.firstinspires.ftc.teamcode.software.Drivetrain drivetrain = new org.firstinspires.ftc.teamcode.software.Drivetrain(
				hardwareMap, matchSettings);
		
		trajectoryEngine = new TrajectoryEngine(limelightManager, matchSettings, drivetrain);
		
		waitForStart();
		
		while (opModeIsActive()) {
			// Get current pitch angle from servo
			double currentPitchDegrees = Settings.Launcher.servoToPitch(pitchServo.getPosition());
			
			// Get aiming solution from trajectory engine
			TrajectoryEngine.AimingSolution solution = trajectoryEngine.getAimingOffsets(
					matchSettings.getAllianceColor(), currentPitchDegrees);
			
			// Get the latest vision results from the Limelight for display
			LLResult result = limelight.getLatestResult();
			
			// Check if the result is valid
			if (result != null && result.isValid()) {
				List<FiducialResult> fiducials = result.getFiducialResults();
				
				boolean tagFound = false;
				for (FiducialResult fiducial : fiducials) {
					// Check if the current fiducial's ID matches our target
					if (fiducial.getFiducialId() == TARGET_TAG_ID) {
						tagFound = true;
						
						// ===== RAW LIMELIGHT DATA =====
						logging.addLine("=== RAW LIMELIGHT DATA ===");
						double tx = fiducial.getTargetXDegrees();
						double ty = fiducial.getTargetYDegrees();
						logging.addData("Target ID", fiducial.getFiducialId());
						logging.addData("TX (Yaw Offset)°", "%.2f", tx);
						logging.addData("TY (Vertical Angle)°", "%.2f", ty);
						logging.addData("Area %%", "%.2f", fiducial.getTargetArea());
						
						// ===== SERVO & CONFIGURATION =====
						logging.addLine("=== SERVO & CONFIGURATION ===");
						logging.addData("Pitch Servo Position", "%.3f", pitchServo.getPosition());
						logging.addData("Current Pitch Angle", "%.1f°", currentPitchDegrees);
						logging.addData("Pitch Axis Height", "%.2f in", Settings.Aiming.PITCH_AXIS_HEIGHT_INCHES);
						logging.addData("AprilTag Height", "%.2f in", Settings.Aiming.APRILTAG_CENTER_HEIGHT_INCHES);
						logging.addData("Target Offset Above Tag", "%.1f in",
								Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES);
						logging.addData("Wheel Speed", "%.0f RPM", Settings.Aiming.WHEEL_SPEED_RPM);
						
						// ===== TRAJECTORY ENGINE SOLUTION =====
						logging.addLine("=== TRAJECTORY ENGINE SOLUTION ===");
						
						if (solution.hasTarget) {
							logging.addData("✓ Target Acquired", true);
							logging.addData("Yaw Correction", "%.2f°", solution.horizontalOffsetDegrees);
							logging.addData("Required Launch Angle", "%.2f°", solution.verticalOffsetDegrees);
							logging.addData("Launch Velocity", "%.1f in/s", solution.launchVelocityInchesPerSec);
							logging.addData("Required RPM", "%.0f", solution.getRequiredWheelSpeedRPM());
							
							// Calculate estimated flight time
							double launchAngleRad = Math.toRadians(solution.verticalOffsetDegrees);
							double vx = solution.launchVelocityInchesPerSec * Math.cos(launchAngleRad);
							
							// Estimate distance (rough calculation for display only)
							double tyActual = Math.abs(ty) < 0.5 ? Math.signum(ty) * 0.5 : ty;
							double absoluteAngle = currentPitchDegrees + tyActual;
							double pitchRad = Math.toRadians(currentPitchDegrees);
							
							// Rough position calculation for flight time estimate
							double llRelY = -Settings.Aiming.LIMELIGHT_FROM_PITCH_AXIS_DOWN_INCHES;
							double llRelX = Settings.Aiming.LIMELIGHT_FROM_PITCH_AXIS_FORWARD_INCHES;
							double rotLLY = llRelX * Math.sin(pitchRad) + llRelY * Math.cos(pitchRad);
							double actualLLY = Settings.Aiming.PITCH_AXIS_HEIGHT_INCHES + rotLLY;
							
							double heightToTag = Settings.Aiming.APRILTAG_CENTER_HEIGHT_INCHES - actualLLY;
							double distFromLL = heightToTag / Math.tan(Math.toRadians(absoluteAngle));
							
							if (distFromLL > 0 && vx > 0) {
								double flightTime = distFromLL / vx;
								logging.addData("Est. Flight Time", "%.2f sec", flightTime);
								logging.addData("Est. Distance", "%.1f in", distFromLL);
							}
							
							// Servo command info
							double targetServoPos = Settings.Launcher.pitchToServo(solution.verticalOffsetDegrees);
							logging.addData("Target Servo Pos", "%.3f", targetServoPos);
							logging.addData("Pitch Change Needed", "%.1f°",
									solution.verticalOffsetDegrees - currentPitchDegrees);
							
							// Alignment status
							logging.addLine("=== ALIGNMENT STATUS ===");
							boolean yawAligned = Math
									.abs(solution.horizontalOffsetDegrees) < Settings.Aiming.MAX_YAW_ERROR;
							boolean pitchAligned = Math.abs(solution.verticalOffsetDegrees
									- currentPitchDegrees) < Settings.Aiming.MAX_PITCH_ERROR;
							
							logging.addData("Yaw Aligned", yawAligned ? "✓ YES" : "✗ NO");
							logging.addData("Pitch Aligned", pitchAligned ? "✓ YES" : "✗ NO");
							logging.addData("Ready to Fire", (yawAligned && pitchAligned) ? "✓ YES" : "✗ NO");
							
						} else {
							logging.addData("✗ No Target", "Check alignment");
						}
						
						// We've found the target, so no need to continue looping
						break;
					}
				}
				
				if (!tagFound) {
					logging.addData("Status", "Looking for Tag %d...", TARGET_TAG_ID);
					logging.addData("Tags Seen", fiducials.size());
					if (!fiducials.isEmpty()) {
						logging.addData("Visible IDs", getVisibleIds(fiducials));
					}
					
					// Still show current pitch even if tag not found
					logging.addLine("=== SERVO STATUS ===");
					logging.addData("Pitch Servo Position", "%.3f", pitchServo.getPosition());
					logging.addData("Current Pitch Angle", "%.1f°", currentPitchDegrees);
				}
			} else {
				logging.addData("Status", "Waiting for Limelight data...");
				if (result != null) {
					logging.addData("LL Valid", result.isValid());
				}
				logging.addData("LL Status", limelight.getStatus());
				
				// Still show current pitch even if no limelight data
				logging.addLine("=== SERVO STATUS ===");
				logging.addData("Pitch Servo Position", "%.3f", pitchServo.getPosition());
				logging.addData("Current Pitch Angle", "%.1f°", currentPitchDegrees);
			}
			
			logging.update();
			sleep(50); // Update 20 times per second
		}
	}
	
	/**
	 * Helper method to show which tags are visible
	 */
	private String getVisibleIds(List<FiducialResult> fiducials) {
		StringBuilder ids = new StringBuilder();
		for (int i = 0; i < fiducials.size(); i++) {
			if (i > 0)
				ids.append(", ");
			ids.append(fiducials.get(i).getFiducialId());
		}
		return ids.toString();
	}
}
