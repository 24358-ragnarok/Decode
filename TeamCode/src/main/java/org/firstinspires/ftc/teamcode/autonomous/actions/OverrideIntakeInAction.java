package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.FlywheelIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Override action that forces the intake motor to start intaking, bypassing
 * normal transfer logic.
 * <p>
 * This action directly controls:
 * - FlywheelIntake: Sets to IN state
 * <p>
 * Useful for testing, debugging, or emergency intake operations where normal
 * transfer state management needs to be overridden.
 * <p>
 * Note: Entrance wheel has been removed. Intake now handles color detection
 * directly.
 */
public class OverrideIntakeInAction implements AutonomousAction {

	/**
	 * Creates an override intake action.
	 */
	public OverrideIntakeInAction() {
	
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		// Start intake motor directly
		FlywheelIntake intake = mechanisms.get(FlywheelIntake.class);
		if (intake != null) {
			intake.in();
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
