package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configuration.Constants;

/**
 * Simplified drivetrain OpMode for outreach and demonstration purposes.
 * Provides basic mecanum drive functionality with gamepad controls and d-pad precision movement.
 * <p>
 * Controls:
 * - Left stick: Drive and strafe
 * - Right stick X: Rotation
 * - Bumpers: Precision rotation (left/right)
 * - D-pad: Precision movement in cardinal directions
 *
 * @noinspection ClassWithoutConstructor
 */
@TeleOp(name = "Run: Drivetrain Only", group = "Outreach")
public class SimpleDrivetrain extends OpMode {
	public DcMotor frontLeftMotor;
	public DcMotor frontRightMotor;
	public DcMotor backLeftMotor;
	public DcMotor backRightMotor;
	
	
	/**
	 * Initializes the drivetrain motors and sets their directions.
	 */
	@Override
	public final void init() {
		frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
		frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
		backLeftMotor = hardwareMap.get(DcMotor.class, "rearLeft");
		backRightMotor = hardwareMap.get(DcMotor.class, "rearRight");
		
		frontRightMotor.setDirection(Constants.driveConstants.rightFrontMotorDirection);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		backRightMotor.setDirection(Constants.driveConstants.rightRearMotorDirection);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		frontLeftMotor.setDirection(Constants.driveConstants.leftFrontMotorDirection);
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		backLeftMotor.setDirection(Constants.driveConstants.leftRearMotorDirection);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
	}
	
	
	/**
	 * Main loop that processes gamepad input and controls the drivetrain.
	 */
	@Override
	public final void loop() {
		double drivePower = -gamepad1.left_stick_y;
		double strafePower = gamepad1.left_stick_x;
		double rotatePower = gamepad1.right_stick_x;
		
		if (gamepad1.right_bumper) {
			rotatePower = -0.1;
		}
		if (gamepad1.left_bumper) {
			rotatePower = 0.1;
		}
		if (gamepad1.dpad_up) {
			drivePower = 0.2;
		}
		if (gamepad1.dpad_down) {
			drivePower = -0.2;
		}
		if (gamepad1.dpad_left) {
			strafePower = -0.2;
		}
		if (gamepad1.dpad_right) {
			strafePower = 0.2;
		}
		
		mecanumDrive(strafePower, drivePower, rotatePower);
	}
	
	/**
	 * Calculates and applies mecanum drive motor powers.
	 *
	 * @param strafePower Left/right movement power (-1.0 to 1.0)
	 * @param drivePower  Forward/backward movement power (-1.0 to 1.0)
	 * @param rotatePower Rotational power (-1.0 to 1.0)
	 */
	public void mecanumDrive(double strafePower, double drivePower, double rotatePower) {
		double frontLeftPower = drivePower + strafePower + rotatePower;
		double frontRightPower = drivePower - strafePower - rotatePower;
		double backLeftPower = drivePower - strafePower + rotatePower;
		double backRightPower = drivePower + strafePower - rotatePower;
		
		frontLeftMotor.setPower(frontLeftPower);
		frontRightMotor.setPower(frontRightPower);
		backLeftMotor.setPower(backLeftPower);
		backRightMotor.setPower(backRightPower);
	}
}