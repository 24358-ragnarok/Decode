package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

/**
 * Override action that forces both the intake motor and transfer entrance wheel
 * to start intaking, bypassing normal transfer logic.
 * <p>
 * This action directly controls:
 * - FlexVectorIntake: Sets to IN state
 * - SingleWheelTransfer entrance wheel: Forces open for intake
 * <p>
 * Useful for testing, debugging, or emergency intake operations where normal
 * transfer state management needs to be overridden.
 */
public class OverrideIntakeInAction implements AutonomousAction {
	
	/**
	 * Creates an override intake action.
	 */
	public OverrideIntakeInAction() {
	
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		// Force transfer entrance wheel to open for intake
		SingleWheelTransfer transfer = mechanisms.get(SingleWheelTransfer.class);
		if (transfer != null) {
			// Use the public forceOpenEntrance method to directly open the entrance wheel
			transfer.forceOpenEntrance();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// This action completes immediately - it just starts the intake override
		// The intake will continue running until stopped by another action or
		// stopOnComplete
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
	}
	
	@Override
	public String getName() {
		return "OverrideIntakeInAction";
	}
}
