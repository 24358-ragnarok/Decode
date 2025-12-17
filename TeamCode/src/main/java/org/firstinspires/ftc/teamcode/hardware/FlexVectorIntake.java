package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.IN_SPEED;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.OUT_SPEED;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.STOPPED_SPEED;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.software.ColorUnifier;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

/**
 * The flex-vector intake controls the robot's intake mechanism using a flywheel motor.
 * Supports intake, outtake, and stop operations with state tracking.
 */
public class FlexVectorIntake extends Mechanism {
	private final DcMotorEx motor;
	private final ColorUnifier unifier;
	private final MechanismManager mechanisms;
	public IntakeState state;
	
	/**
	 * Creates a new FlexVectorIntake instance.
	 *
	 * @param intakeMotor The DC motor that controls the intake flywheel
	 */
	public FlexVectorIntake(MechanismManager mechanisms, DcMotorEx intakeMotor, ColorUnifier color) {
		this.mechanisms = mechanisms;
		this.motor = intakeMotor;
		this.unifier = color;
		this.state = IntakeState.STOPPED;
	}
	
	/**
	 * Starts the intake motor to pull artifacts in.
	 */
	public void in() {
		state = IntakeState.IN;
		motor.setPower(IN_SPEED);
	}
	
	/**
	 * Reverses the intake motor to push artifacts out.
	 */
	public void out() {
		state = IntakeState.OUT;
		motor.setPower(OUT_SPEED);
	}
	
	/**
	 * Stops the intake motor.
	 */
	public void stop() {
		state = IntakeState.STOPPED;
		motor.setPower(STOPPED_SPEED);
	}
	
	/**
	 * The Intake should begin stopped.
	 */
	@Override
	public void start() {
		stop();
	}
	
	/**
	 * Updates the intake mechanism to sense new balls.
	 */
	@Override
	public void update() {
		Artifact detection = unifier.find();
		if (!detection.equals(Artifact.NONE)) {
			mechanisms.ifValid(mechanisms.get(VerticalWheelTransfer.class), vwt -> {
				vwt.artifactIncoming(detection);
			});
		}
	}
	
	/**
	 * Represents the current state of the intake mechanism.
	 */
	public enum IntakeState {
		/**
		 * Intake motor running forward to pull artifacts in
		 */
		IN,
		/**
		 * Intake motor running in reverse to push artifacts out
		 */
		OUT,
		/**
		 * Intake motor stopped
		 */
		STOPPED
	}
}