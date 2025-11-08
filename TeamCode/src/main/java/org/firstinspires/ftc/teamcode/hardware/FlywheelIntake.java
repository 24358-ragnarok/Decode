package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * FlywheelIntake controls the robot's intake mechanism using a flywheel motor.
 * Supports intake, outtake, and stop operations with state tracking.
 */
public class FlywheelIntake extends Mechanism {
	private final DcMotor intakeMotor;
	public IntakeState state;
	
	/**
	 * Creates a new FlywheelIntake instance.
	 *
	 * @param intakeMotor The DC motor that controls the intake flywheel
	 */
	public FlywheelIntake(DcMotor intakeMotor) {
		this.intakeMotor = intakeMotor;
		this.state = IntakeState.STOPPED;
	}
	
	/**
	 * Starts the intake motor to pull artifacts in.
	 */
	public void in() {
		state = IntakeState.IN;
		intakeMotor.setPower(SPEED);
	}
	
	/**
	 * Reverses the intake motor to push artifacts out.
	 */
	public void out() {
		state = IntakeState.OUT;
		intakeMotor.setPower(-SPEED);
	}
	
	/**
	 * Stops the intake motor.
	 */
	public void stop() {
		state = IntakeState.STOPPED;
		intakeMotor.setPower(0);
	}
	
	/**
	 * Initializes the intake by starting the motor.
	 */
	@Override
	public void start() {
		stop();
	}
	
	/**
	 * Toggles between intake and stopped states.
	 */
	public void toggleIn() {
		if (state == IntakeState.IN) {
			stop();
		} else {
			in();
		}
	}
	
	/**
	 * Toggles between outtake and stopped states.
	 */
	public void toggleOut() {
		if (state == IntakeState.OUT) {
			stop();
		} else {
			out();
		}
	}
	
	/**
	 * Updates the intake mechanism. Currently no periodic updates needed.
	 */
	@Override
	public void update() {
		// No periodic updates required for this mechanism
	}
	
	/**
	 * Represents the current state of the intake mechanism.
	 */
	public enum IntakeState {
		/** Intake motor running forward to pull artifacts in */
		IN,
		/** Intake motor running in reverse to push artifacts out */
		OUT,
		/** Intake motor stopped */
		STOPPED
	}
}