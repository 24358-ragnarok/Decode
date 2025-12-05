package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.FlywheelIntake;
import org.firstinspires.ftc.teamcode.software.ColorSensor;

/**
 * Test OpMode for FlywheelIntake with detailed debug output.
 * <p>
 * Tracks ball detection timing:
 * - Shows current intake state and detected color
 * - Displays "Time Last Ball Sensed" - duration from ball detection to ball exit
 * - Tracks transitions: UNKNOWN -> COLOR (ball detected) and COLOR -> UNKNOWN (ball left)
 * <p>
 * Controls:
 * - Right Trigger: Intake in
 * - Left Trigger: Intake out
 * - A Button: Toggle intake in/stop
 * - B Button: Toggle intake out/stop
 */
@TeleOp(name = "Test: Flywheel Intake", group = "Tests")
public class FlywheelIntakeTest extends OpMode {
	private FlywheelIntake intake;
	private ColorSensor colorSensor;
	private RevColorSensorV3 rawSensor; // For direct access to raw values
	
	// Ball detection tracking
	private MatchSettings.ArtifactColor currentDetectedColor = MatchSettings.ArtifactColor.UNKNOWN;
	private MatchSettings.ArtifactColor previousDetectedColor = MatchSettings.ArtifactColor.UNKNOWN;
	private long ballDetectedTimeMs = 0;
	private long ballExitedTimeMs = 0;
	private long lastBallSensedDurationMs = 0; // Duration from detection to exit
	private boolean ballCurrentlyDetected = false;
	
	@Override
	public void init() {
		// Initialize hardware
		DcMotorEx intakeMotor = Settings.Hardware.INTAKE_MOTOR.fromHardwareMap(hardwareMap);
		rawSensor = Settings.Hardware.TRANSFER_COLOR_SENSOR.fromHardwareMap(hardwareMap);
		colorSensor = new ColorSensor(rawSensor);
		
		// Create intake without transfer (standalone test)
		intake = new FlywheelIntake(intakeMotor, colorSensor);
		intake.start();
		
		telemetry.addLine("✅ FlywheelIntake Test Initialized");
		telemetry.addLine();
		telemetry.addLine("Controls:");
		telemetry.addLine("  Right Trigger: Intake IN");
		telemetry.addLine("  Left Trigger: Intake OUT");
		telemetry.addLine("  A Button: Toggle IN/Stop");
		telemetry.addLine("  B Button: Toggle OUT/Stop");
		telemetry.update();
	}
	
	@Override
	public void loop() {
		// Update intake mechanism (handles color detection internally)
		intake.update();
		
		// Handle controls
		if (gamepad1.right_trigger > 0.1) {
			intake.in();
		} else if (gamepad1.left_trigger > 0.1) {
			intake.out();
		}
		
		if (gamepad1.a) {
			intake.toggleIn();
		}
		if (gamepad1.b) {
			intake.toggleOut();
		}
		
		// Poll color sensor directly for debug display (bypassing debounce for real-time feedback)
		MatchSettings.ArtifactColor detected = colorSensor.getArtifactColor();
		currentDetectedColor = detected;
		
		// Track ball detection transitions
		long now = System.currentTimeMillis();
		
		// Detect transition: UNKNOWN -> COLOR (ball detected)
		if (previousDetectedColor == MatchSettings.ArtifactColor.UNKNOWN &&
				currentDetectedColor != MatchSettings.ArtifactColor.UNKNOWN) {
			ballDetectedTimeMs = now;
			ballCurrentlyDetected = true;
		}
		
		// Detect transition: COLOR -> UNKNOWN (ball exited)
		if (previousDetectedColor != MatchSettings.ArtifactColor.UNKNOWN &&
				currentDetectedColor == MatchSettings.ArtifactColor.UNKNOWN) {
			ballExitedTimeMs = now;
			ballCurrentlyDetected = false;
			
			// Calculate and save duration
			if (ballDetectedTimeMs > 0) {
				lastBallSensedDurationMs = ballExitedTimeMs - ballDetectedTimeMs;
			}
		}
		
		previousDetectedColor = currentDetectedColor;
		
		// Display debug information
		telemetry.clear();
		telemetry.addLine("═══════════════════════════════════");
		telemetry.addLine("  FLYWHEEL INTAKE DEBUG");
		telemetry.addLine("═══════════════════════════════════");
		telemetry.addLine();
		
		// Intake state
		telemetry.addData("Intake State", intake.state);
		telemetry.addLine();
		
		// Color detection
		telemetry.addLine("─── Color Detection ───");
		telemetry.addData("Detected Color", currentDetectedColor);
		telemetry.addData("Ball Currently Detected", ballCurrentlyDetected ? "YES" : "NO");
		
		if (ballCurrentlyDetected && ballDetectedTimeMs > 0) {
			long timeSinceDetection = now - ballDetectedTimeMs;
			telemetry.addData("Time Since Detection", "%.2f s", timeSinceDetection / 1000.0);
		}
		telemetry.addLine();
		
		// Time Last Ball Sensed
		telemetry.addLine("─── Time Last Ball Sensed ───");
		if (lastBallSensedDurationMs > 0) {
			telemetry.addData("Duration", "%.3f s", lastBallSensedDurationMs / 1000.0);
			telemetry.addData("Duration (ms)", "%d ms", lastBallSensedDurationMs);
		} else {
			telemetry.addData("Duration", "No ball sensed yet");
		}
		telemetry.addLine();
		
		// Raw sensor values
		telemetry.addLine("─── Raw Sensor Values ───");
		telemetry.addData("Red", "%d", rawSensor.red());
		telemetry.addData("Green", "%d", rawSensor.green());
		telemetry.addData("Blue", "%d", rawSensor.blue());
		telemetry.addLine();
		
		// Settings reference
		telemetry.addLine("─── Settings ───");
		telemetry.addData("Debounce Time", "%d ms", Settings.Intake.COLOR_DETECTION_DEBOUNCE_MS);
		telemetry.addData("Travel Time", "%d ms", Settings.Intake.BALL_TRAVEL_TIME_MS);
		telemetry.addData("Intake Speed", "%f", Settings.Intake.SPEED);
		
		telemetry.update();
	}
	
	@Override
	public void stop() {
		if (intake != null) {
			intake.stop();
		}
	}
}

