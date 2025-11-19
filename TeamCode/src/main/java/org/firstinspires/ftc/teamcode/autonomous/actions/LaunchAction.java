package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.EXIT_FIRE_DURATION_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.EXIT_FIRE_RESET_MS;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.PairedLauncher;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;

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
public class LaunchAction implements AutonomousAction {
	private boolean hasTransfer;
	private boolean hasLauncher;
	private State state;
	private long lastFireTimeMs;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		hasLauncher = mechanisms.get(PairedLauncher.class) != null;
		hasTransfer = mechanisms.get(VerticalWheelTransfer.class) != null;
		state = State.READY_TO_LAUNCH;
		lastFireTimeMs = 0;
		
		// Start the launcher ready sequence (spin up, aim)
		if (hasLauncher) {
			PairedLauncher launcher = mechanisms.get(PairedLauncher.class);
			launcher.ready();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// If no parts, we can't launch anything
		if (!hasTransfer || !hasLauncher) {
			return true;
		}
		
		PairedLauncher launcher = mechanisms.get(PairedLauncher.class);
		VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
		
		if (transfer.isEmpty() && state == State.READY_TO_LAUNCH) {
			state = State.COMPLETE; // nothing to fire initially, just quit
		}
		
		// Maintain launcher ready state (spin-up and aim)
		if (hasLauncher && launcher != null) {
			launcher.maintainReady();
		}
		
		// State machine for sequential ball firing
		switch (state) {
			case READY_TO_LAUNCH:
				state = State.ADVANCING_BALL;
				break;
			
			case ADVANCING_BALL:
				// Check if transfer is empty and done with previous launch
				if (transfer.isEmpty() && System.currentTimeMillis() - lastFireTimeMs > EXIT_FIRE_DURATION_MS) {
					state = State.COMPLETE;
					break;
				}
				
				if (System.currentTimeMillis() - lastFireTimeMs > EXIT_FIRE_DURATION_MS + EXIT_FIRE_RESET_MS) {
					transfer.moveNextArtifactToLauncher();
					state = State.WAITING_TO_FIRE;
				}
				break;
			
			case WAITING_TO_FIRE:
				if (transfer.artifactInFiringPosition()) {
					state = State.FIRING;
				}
				if (transfer.isEmpty()) {
					state = State.COMPLETE;
				}
				break;
			
			case FIRING:
				// Fire the ball
				launcher.fire();
				lastFireTimeMs = System.currentTimeMillis();
				// Go back to advancing the next ball
				state = State.ADVANCING_BALL;
				break;
			
			case COMPLETE:
				// All done
				break;
		}
		
		if (state == State.COMPLETE) {
			launcher.stop();
			return true;
		} else {
			return false;
		}
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// If interrupted, stop the launcher
		if (interrupted && hasLauncher) {
			PairedLauncher launcher = mechanisms.get(PairedLauncher.class);
			if (launcher != null) {
				launcher.stop();
			}
		}
		// Otherwise, launcher continues running for next action
	}
	
	@Override
	public String getName() {
		return "Launch, " + state.toString();
	}
	
	private enum State {
		READY_TO_LAUNCH, // Waiting for launcher spin-up
		ADVANCING_BALL, // Moving next ball to kicker
		WAITING_TO_FIRE, // Ball at kicker, waiting for cooldown
		FIRING, // Currently firing
		COMPLETE // All balls fired
	}
}
