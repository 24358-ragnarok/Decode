package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.HorizontalLauncher;

/**
 * An enhanced TeleOp for testing launcher motor performance and angle.
 * This OpMode allows for fine-tuning of motor speed and provides real-time
 * telemetry for motor RPM and the launcher's physical angle, which are
 * crucial for projectile motion calculations.
 *
 * @noinspection ClassWithoutConstructor, OverlyLongMethod
 */
@TeleOp(name = "Test: Outtake", group = "Tests")
public class ProjectileMotionTest extends LinearOpMode {
	
	// Motor Configuration
	final double TICKS_PER_REVOLUTION = 28;
	// State & Control
	private double commandedRPM = 3000;
	// Hardware
	private HorizontalLauncher.SyncBelt syncBelt;
	private DcMotorEx rightLauncherMotor;
	private DcMotorEx leftLauncherMotor;
	private Servo kickerServo;
	private Servo pitchServo;
	private IMU imu; // The Inertial Measurement Unit
	private double commandedAngle = Settings.Launcher.DEFAULT_PITCH_ANGLE;
	
	@Override
	public final void runOpMode() {
		// --- Motor Initialization ---
		rightLauncherMotor = hardwareMap.get(DcMotorEx.class, Settings.HardwareIDs.LAUNCHER_RIGHT);
		leftLauncherMotor = hardwareMap.get(DcMotorEx.class, Settings.HardwareIDs.LAUNCHER_LEFT);
		
		rightLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
		
		syncBelt = new HorizontalLauncher.SyncBelt(rightLauncherMotor, leftLauncherMotor);
		pitchServo = hardwareMap.get(Servo.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
		pitchServo.setPosition(commandedAngle);
		kickerServo = hardwareMap.get(Servo.class, Settings.HardwareIDs.TRANSFER_EXIT_KICKER);
		kickerServo.setPosition(Settings.Transfer.EXIT_LOCK_POSITION);
		
		telemetry.addLine("✅ Initialization Complete");
		telemetry.addLine("Controls:");
		telemetry.addLine("  DPAD Up&Down: Adjust Pitch");
		telemetry.addLine("  BUMPER L/R: Adjust Speed up/down");
		telemetry.addLine("  Left Trigger: Spin up launcher");
		telemetry.addLine("  Right Trigger: Kick");
		telemetry.update();
		
		waitForStart();
		
		// --- Main Loop ---
		while (opModeIsActive()) {
			// --- Gamepad Input for fine-tuning ---
			if (gamepad1.dpad_up) {
				commandedAngle -= 0.001;
			}
			if (gamepad1.dpad_down) {
				commandedAngle += 0.001;
			}
			if (gamepad1.left_bumper) {
				commandedRPM -= 5;
			}
			if (gamepad1.right_bumper) {
				commandedRPM += 5;
			}
			if (gamepad1.right_trigger > 0.1) {
				kickerServo.setPosition(Settings.Transfer.EXIT_KICK_POSITION);
			} else {
				kickerServo.setPosition(Settings.Transfer.EXIT_LOCK_POSITION);
			}
			
			commandedRPM = Math.max(0, Math.min(6000, commandedRPM));
			commandedAngle = Math.max(Settings.Launcher.PITCH_SERVO_AT_MAX, Math.min(Settings.Launcher.PITCH_SERVO_AT_MIN, commandedAngle));
			
			if (gamepad1.left_trigger > 0.1) {
				syncBelt.spinUpToRPM(commandedRPM);
			} else {
				syncBelt.spinDown();
			}
			pitchServo.setPosition(commandedAngle);
			
			// --- Sensor Readings ---
			double rightRPM = (rightLauncherMotor.getVelocity() / TICKS_PER_REVOLUTION) * 60;
			double leftRPM = (leftLauncherMotor.getVelocity() / TICKS_PER_REVOLUTION) * 60;
			
			// --- Telemetry ---
			telemetry.addData("Commanded RPM", commandedRPM);
			telemetry.addData("Real Average RPM", "%.2f", (rightRPM + leftRPM) / 2.0);
			
			telemetry.addLine();
			
			telemetry.addData("Commanded Pitch Position", commandedAngle);
			telemetry.addData("Real Angle", HorizontalLauncher.getPitchDirect(pitchServo));
			
			telemetry.update();
		}
	}
}