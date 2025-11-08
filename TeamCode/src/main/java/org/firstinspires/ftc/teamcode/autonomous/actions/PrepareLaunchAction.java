package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.BoonstraBlaster;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

/**
 * Action that prepares both the transfer and launcher for an upcoming launch.
 * This spins up the launcher belt early so it's already at speed when
 * LaunchAction executes,
 * and tells the transfer to prepare the first ball.
 */
public class PrepareLaunchAction implements AutonomousAction {
	private boolean hasTransfer;
	private boolean hasLauncher;

	@Override
	public void initialize(MechanismManager mechanisms) {
		hasTransfer = mechanisms.get(SingleWheelTransfer.class) != null;
		hasLauncher = mechanisms.get(BoonstraBlaster.class) != null;
	}

	@Override
	public boolean execute(MechanismManager mechanisms) {
		// Start spinning up launcher if available
		if (hasLauncher) {
			BoonstraBlaster launcher = mechanisms.get(BoonstraBlaster.class);
			launcher.ready();
		}

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
