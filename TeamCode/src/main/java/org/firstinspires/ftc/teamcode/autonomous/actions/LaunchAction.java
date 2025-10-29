package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_FIRE_DURATION_MS;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.hardware.HorizontalLauncher;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

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
		hasLauncher = mechanisms.get(HorizontalLauncher.class) != null;
		hasTransfer = mechanisms.get(SingleWheelTransfer.class) != null;
		state = State.READY_TO_LAUNCH;
		lastFireTimeMs = 0;
		
		// Start the launcher ready sequence (spin up, aim)
		if (hasLauncher) {
			HorizontalLauncher launcher = mechanisms.get(HorizontalLauncher.class);
			launcher.ready();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// If no transfer, we can't launch anything
		if (!hasTransfer) {
			return true;
		}
		
		HorizontalLauncher launcher = mechanisms.get(HorizontalLauncher.class);
		SingleWheelTransfer transfer = mechanisms.get(SingleWheelTransfer.class);
		
		// Maintain launcher ready state (spin-up and aim)
		if (hasLauncher && launcher != null) {
			launcher.maintainReady();
		}
		
		// State machine for sequential ball firing
		switch (state) {
			case READY_TO_LAUNCH:
				// Check if launcher is ready to fire
				boolean launcherReady = !hasLauncher || (launcher != null && launcher.okayToLaunch());
				if (launcherReady) {
					state = State.ADVANCING_BALL;
				}
				break;
			
			case ADVANCING_BALL:
				// Check if transfer is empty
				if (transfer.isEmpty()) {
					state = State.COMPLETE;
					break;
				}
				
				// Find the next ball needed and advance it to the kicker
				MatchSettings.ArtifactColor nextNeeded = mechanisms.matchSettings.nextArtifactNeeded();
				
				// If we don't know what we need, just advance whatever is there
				if (nextNeeded == MatchSettings.ArtifactColor.UNKNOWN) {
					// Just fire whatever is at the kicker or advance the first ball
					if (transfer.getNextBallToKicker() == MatchSettings.ArtifactColor.UNKNOWN) {
						// Nothing at kicker, advance first ball
						transfer.advance();
					}
					state = State.WAITING_TO_FIRE;
					break;
				}
				
				// Look for the needed ball in the transfer
				int ballIndex = transfer.indexOf(nextNeeded);
				if (ballIndex >= 0) {
					// Found the ball, move it to kicker
					transfer.moveSlotToKicker(ballIndex);
				} else {
					// Don't have the needed ball, fire whatever we have
					if (transfer.getNextBallToKicker() == MatchSettings.ArtifactColor.UNKNOWN) {
						transfer.advance();
					}
				}
				state = State.WAITING_TO_FIRE;
				break;
			
			case WAITING_TO_FIRE:
				// Wait for wheel to stop running and cooldown period
				long now = System.currentTimeMillis();
				boolean wheelStopped = !transfer.isTransferWheelRunning();
				boolean cooldownPassed = (now - lastFireTimeMs) > EXIT_FIRE_DURATION_MS;
				
				if (wheelStopped && cooldownPassed) {
					// Ball should be at kicker now, check if there's something to fire
					if (transfer.getNextBallToKicker() != MatchSettings.ArtifactColor.UNKNOWN) {
						state = State.FIRING;
					} else {
						// Nothing to fire, we must be empty
						state = State.COMPLETE;
					}
				}
				break;
			
			case FIRING:
				// Fire the ball
				transfer.fire();
				lastFireTimeMs = System.currentTimeMillis();
				// Go back to advancing the next ball
				state = State.ADVANCING_BALL;
				break;
			
			case COMPLETE:
				// All done
				break;
		}
		
		// Complete when transfer is empty or we're in COMPLETE state
		return transfer.isEmpty() || state == State.COMPLETE;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// If interrupted, stop the launcher
		if (interrupted && hasLauncher) {
			HorizontalLauncher launcher = mechanisms.get(HorizontalLauncher.class);
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
	
	private enum State {
		READY_TO_LAUNCH, // Waiting for launcher spin-up
		ADVANCING_BALL, // Moving next ball to kicker
		WAITING_TO_FIRE, // Ball at kicker, waiting for cooldown
		FIRING, // Currently firing
		COMPLETE // All balls fired
	}
}
