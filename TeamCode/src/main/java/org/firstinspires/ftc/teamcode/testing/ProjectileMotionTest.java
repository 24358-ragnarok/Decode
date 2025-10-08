package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
	final double TICKS_PER_REVOLUTION = 1120;
	// Hardware
	private HorizontalLauncher.SyncBelt syncBelt;
	private DcMotorEx rightLauncherMotor;
	private DcMotorEx leftLauncherMotor;
	private IMU imu; // The Inertial Measurement Unit
	// State & Control
	private double commandedMotorSpeed = 1.0;
	
	@Override
	public final void runOpMode() {
		// --- Motor Initialization ---
		rightLauncherMotor = hardwareMap.get(DcMotorEx.class, Settings.HardwareIDs.LAUNCHER_RIGHT);
		leftLauncherMotor = hardwareMap.get(DcMotorEx.class, Settings.HardwareIDs.LAUNCHER_LEFT);
		
		rightLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
		syncBelt = new HorizontalLauncher.SyncBelt(rightLauncherMotor, leftLauncherMotor);
		
		// --- IMU Initialization ---
		imu = hardwareMap.get(IMU.class, "imu");
		
		/*
		 * ❗ IMPORTANT: Define the orientation of the Control Hub on your robot.
		 * This is essential for the IMU to report correct angles.
		 * Below, we assume the hub is laid flat (logo facing UP) with the USB ports
		 * facing the FORWARD direction of the robot.
		 *
		 * Adjust these two parameters to match your robot's configuration.
		 */
		RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
		RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
		RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
		
		imu.initialize(new IMU.Parameters(orientationOnRobot));
		
		telemetry.addLine("✅ Initialization Complete");
		telemetry.addData("IMU initial angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		telemetry.addLine("------------------------------------");
		telemetry.addLine("Controls:");
		telemetry.addLine("  DPAD Up&Down: Adjust Speed");
		telemetry.addLine("  Left Trigger: Spin up launcher");
		telemetry.update();
		
		waitForStart();
		
		// --- Main Loop ---
		while (opModeIsActive()) {
			// --- Gamepad Input for fine-tuning ---
			if (gamepad1.dpad_up) {
				commandedMotorSpeed += 0.005;
			}
			if (gamepad1.dpad_down) {
				commandedMotorSpeed -= 0.005;
			}
			commandedMotorSpeed = Math.max(-1.0, Math.min(1.0, commandedMotorSpeed));
			
			if (gamepad1.aWasPressed()) {
				syncBelt.spinUp(commandedMotorSpeed);
			}
			
			if (gamepad1.bWasPressed()) {
				syncBelt.spinDown();
			}
			
			// --- Sensor Readings ---
			double rightRPM = (rightLauncherMotor.getVelocity() / TICKS_PER_REVOLUTION) * 60;
			double leftRPM = (leftLauncherMotor.getVelocity() / TICKS_PER_REVOLUTION) * 60;
			
			YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
			double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
			
			// --- Telemetry ---
			telemetry.addData("Commanded Power", commandedMotorSpeed);
			telemetry.addData("Average RPM", "%.2f", (rightRPM + leftRPM) / 2.0);
			telemetry.addData("Launcher Angle", yaw);
			
			telemetry.update();
		}
	}
}