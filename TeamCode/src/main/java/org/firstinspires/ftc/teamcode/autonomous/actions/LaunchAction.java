package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.Spindex;

/**
 * Action that launches all samples using the launcher mechanism.
 * <p>
 * This action coordinates between the launcher and spindex:
 * 1. Spins up the launcher belt and aims
 * 2. Triggers the spindex rapid fire sequence to eject all samples
 * 3. Waits for the spindex to be completely empty
 * <p>
 * The action continues calling ready() on the launcher to maintain
 * spin-up and aiming throughout the launch sequence.
 */
public class LaunchAction implements AutonomousAction {
	private boolean hasSpindex;
	private boolean hasLauncher;
	private boolean rapidFireTriggered;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		hasLauncher = mechanisms.get(Launcher.class) != null;
		hasSpindex = mechanisms.get(Spindex.class) != null;
		rapidFireTriggered = false;
		
		// Start the launcher ready sequence (spin up, aim)
		if (hasLauncher) {
			Launcher launcher = mechanisms.get(Launcher.class);
			launcher.ready();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// If no spindex, we can't launch anything
		if (!hasSpindex) {
			return true;
		}
		
		Launcher launcher = mechanisms.get(Launcher.class);
		Spindex spindex = mechanisms.get(Spindex.class);
		
		// Keep the launcher ready (maintains spin-up and aiming)
		if (hasLauncher && launcher != null) {
			launcher.ready();
		}
		
		// Once ready, trigger rapid fire (only once)
		if (!rapidFireTriggered) {
			boolean readyToFire = !hasLauncher || (launcher != null && launcher.okayToLaunch());
			if (readyToFire && spindex != null) {
				spindex.rapidFireSequence();
				rapidFireTriggered = true;
			}
		}
		
		// Complete when spindex is empty
		return spindex == null || spindex.isEmpty();
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// If interrupted, stop the launcher
		if (interrupted && hasLauncher) {
			Launcher launcher = mechanisms.get(Launcher.class);
			if (launcher != null) {
				launcher.stop();
			}
		}
		// Otherwise, launcher continues running for next action
	}
	
	@Override
	public String getName() {
		return "Launch";
	}
}
