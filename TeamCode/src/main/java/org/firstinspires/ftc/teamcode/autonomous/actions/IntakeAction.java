package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.Spindex;

/**
 * Action that runs the intake mechanism.
 * <p>
 * This action coordinates both the Intake motor and the Spindex:
 * - Tells the Spindex to prepare for intake (rotate empty slot, open seal)
 * - Starts the intake motor to pull samples in
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
		// Prepare spindex: find empty slot, rotate to intake, open seal
		Spindex spindex = mechanisms.get(Spindex.class);
		if (spindex != null) {
			spindex.prepareForIntake();
		}
		
		// Start intake motor to pull samples in
		Intake intake = mechanisms.get(Intake.class);
		if (intake != null) {
			intake.in();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// This action completes immediately - it just starts the intake
		// The intake will continue running until stopped by another action
		// The spindex IntakeCommand runs asynchronously in the spindex update loop
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		if (stopOnComplete) {
			Intake intake = mechanisms.get(Intake.class);
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
