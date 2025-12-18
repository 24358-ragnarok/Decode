package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.DEFAULT_PITCH_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.PairedLauncher;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;

/**
 * An enhanced TeleOp for testing launcher motor performance and angle.
 * This OpMode allows for fine-tuning of motor speed and provides real-time
 * telemetry for motor RPM and the launcher's physical angle, which are
 * crucial for projectile motion calculations.
 *
 * @noinspection ClassWithoutConstructor, OverlyLongMethod
 */
@TeleOp(name = "Test: Outtake", group = "Tests")
public class OuttakeTest extends LinearOpMode {
	
	// State & Control
	private double commandedRPM = 3000;
	private double commandedAngle = DEFAULT_PITCH_ANGLE; // Now in degrees, not servo position
	
	@Override
	public final void runOpMode() {
		MechanismManager m = new MechanismManager(hardwareMap);
		
		// Set initial pitch via PairedLauncher
		m.ifValid(m.get(PairedLauncher.class), pairedLauncher -> {
			pairedLauncher.setPitch(commandedAngle);
		});
		
		telemetry.addLine("✅ Initialization Complete");
		telemetry.addLine("Controls:");
		telemetry.addLine("  DPAD Up&Down: Adjust Pitch (degrees)");
		telemetry.addLine("  BUMPER L/R: Adjust Speed up/down");
		telemetry.addLine("  A: Spin up launcher + open gate");
		telemetry.addLine("  B: Advance transfer");
		telemetry.setMsTransmissionInterval(50);
		telemetry.update();
		
		waitForStart();
		
		// --- Main Loop ---
		while (opModeIsActive()) {
			m.update(); // Required for mechanisms to apply motor commands
			
			// --- Gamepad Input for fine-tuning ---
			if (gamepad1.dpad_up) {
				commandedAngle += 0.1; // Degrees increment
			}
			if (gamepad1.dpad_down) {
				commandedAngle -= 0.1; // Degrees decrement
			}
			if (gamepad1.left_bumper) {
				commandedRPM -= 5;
			}
			if (gamepad1.right_bumper) {
				commandedRPM += 5;
			}
			
			commandedRPM = Math.max(0, Math.min(6000, commandedRPM));
			
			if (gamepad1.aWasPressed()) {
				m.ifValid(m.get(PairedLauncher.class), pairedLauncher -> {
					pairedLauncher.setRPM(commandedRPM);
					pairedLauncher.setPitch(commandedAngle);
					pairedLauncher.spinUp();
					pairedLauncher.open();
				});
			} else if (gamepad1.aWasReleased()) {
				m.ifValid(m.get(PairedLauncher.class), PairedLauncher::spinDown);
			}
			if (gamepad1.bWasPressed()) {
				m.ifValid(m.get(VerticalWheelTransfer.class), VerticalWheelTransfer::advance);
			}
			
			// --- Telemetry ---
			telemetry.addData("Commanded Motor RPM", "%.0f (%.0f wheel RPM)", commandedRPM, commandedRPM * 2.0 / 3.0);
			telemetry.addData("Commanded Angle", "%.1f", commandedAngle);
			
			m.ifValid(m.get(PairedLauncher.class), pairedLauncher -> {
				telemetry.addData("Smoothed RPM", "%.2f", pairedLauncher.getRPM());
				telemetry.addData("Servo Position", "%.2f", Settings.Launcher.pitchToServo(pairedLauncher.getPitch()));
				telemetry.addData("At speed?", pairedLauncher.isAtSpeed());
			});
			
			telemetry.update();
		}
	}
}