package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.HorizontalLauncher;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Action that launches all samples using the launcher mechanism.
 * <p>
 * This action coordinates between the launcher and transfer:
 * 1. Spins up the launcher belt and aims
 * 2. For each ball in the transfer:
 * - Advances the next needed ball to the kicker position
 * - Waits for launcher to be ready
 * - Fires the ball
 * 3. Repeats until the transfer is completely empty
 * <p>
 * The action maintains the launcher in a ready state (spinning and aimed)
 * throughout the launch sequence.
 */
public class SpinUpAction implements AutonomousAction {
	private boolean hasLauncher;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		hasLauncher = mechanisms.get(HorizontalLauncher.class) != null;
		
		// Start the launcher ready sequence (spin up, aim)
		if (hasLauncher) {
			HorizontalLauncher launcher = mechanisms.get(HorizontalLauncher.class);
			launcher.spinUp();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		HorizontalLauncher launcher = mechanisms.get(HorizontalLauncher.class);
		
		// Maintain launcher ready state (spin-up and aim)
		if (hasLauncher && launcher != null) {
			launcher.maintainReady();
		}
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
	}
	
	@Override
	public String getName() {
		return "Spin Up";
	}
	
}
