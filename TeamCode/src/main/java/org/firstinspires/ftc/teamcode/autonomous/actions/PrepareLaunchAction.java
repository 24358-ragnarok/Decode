package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

/**
 * Action that tells the transfer to asynchronously prepare for launch.
 */
public class PrepareLaunchAction implements AutonomousAction {
	private boolean hasTransfer;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		hasTransfer = mechanisms.get(SingleWheelTransfer.class) != null;
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// If no transfer, we can't launch anything
		if (!hasTransfer) {
			return true;
		}
		SingleWheelTransfer transfer = mechanisms.get(SingleWheelTransfer.class);
		
		transfer.prepareToLaunch();
		
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
	}
	
	@Override
	public String getName() {
		return "Prepare Launch";
	}
}
