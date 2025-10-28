package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.FlywheelIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Action that stops the intake mechanism.
 */
public class StopIntakeAction implements AutonomousAction {
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		FlywheelIntake intake = mechanisms.get(FlywheelIntake.class);
		if (intake != null) {
			intake.stop();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		return true; // Completes immediately
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// Nothing to clean up
	}
	
	@Override
	public String getName() {
		return "StopIntake";
	}
}
