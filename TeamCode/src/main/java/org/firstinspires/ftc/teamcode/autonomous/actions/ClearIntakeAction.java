package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.FlywheelIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

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
public class ClearIntakeAction implements AutonomousAction {
	private final boolean stopOnComplete;

	public ClearIntakeAction(boolean stopOnComplete) {
		this.stopOnComplete = stopOnComplete;
	}

	public ClearIntakeAction() {
		this(false);
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		// Start outtake to clear balls from intake
		SingleWheelTransfer transfer = mechanisms.get(SingleWheelTransfer.class);
		if (transfer != null) {
			transfer.clearEntrance();
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
			// Stop outtake when clearing is complete
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
