package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;

/**
 * Action that stops the intake mechanism.
 */
public class EndPickupAction implements AutonomousAction {
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		FlexVectorIntake intake = mechanisms.get(FlexVectorIntake.class);
		if (intake != null) {
			intake.stop();
		}
		VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
		if (transfer != null) {
			transfer.freeze();
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
