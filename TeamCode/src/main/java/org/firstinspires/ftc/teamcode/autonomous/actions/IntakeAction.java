package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.FlywheelIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Action that runs the intake mechanism.
 * <p>
 * This action coordinates both the Intake motor and the Transfer:
 * - Starts the intake motor to pull samples in
 * - The transfer automatically detects and stores incoming balls via its color
 * sensor
 * - Transfer automatically advances balls as they are detected
 * <p>
 * Can be configured to stop on complete or leave running.
 */
public class IntakeAction implements AutonomousAction {
	private final boolean stopOnComplete;
	
	public IntakeAction(boolean stopOnComplete) {
		this.stopOnComplete = stopOnComplete;
	}
	
	public IntakeAction() {
		this(false);
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		// Transfer doesn't need preparation - it automatically detects and stores balls
		// The SingleWheelTransfer update() loop handles color detection and advancement
		
		// Start intake motor to pull samples in
		FlywheelIntake intake = mechanisms.get(FlywheelIntake.class);
		if (intake != null) {
			intake.in();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// This action completes immediately - it just starts the intake
		// The intake will continue running until stopped by another action
		// The transfer automatically detects and advances balls in its update loop
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		if (stopOnComplete) {
			FlywheelIntake intake = mechanisms.get(FlywheelIntake.class);
			if (intake != null) {
				intake.stop();
			}
		}
	}
	
	@Override
	public String getName() {
		return "Intake" + (stopOnComplete ? "AndStop" : "Start");
	}
}
