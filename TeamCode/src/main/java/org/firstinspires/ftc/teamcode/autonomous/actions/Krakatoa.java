package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.KRAKATOA_TIME_MS;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
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
public class Krakatoa implements AutonomousAction {
	private boolean hasTransfer;
	private boolean hasLauncher;
	private boolean hasIntake;
	private State state;
	private Timer krakaTimer;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		hasLauncher = mechanisms.get(PairedLauncher.class) != null;
		hasTransfer = mechanisms.get(VerticalWheelTransfer.class) != null;
		hasIntake = mechanisms.get(FlexVectorIntake.class) != null;
		state = State.WAITING_TO_FIRE;
		krakaTimer = new Timer();
		
		// Start the launcher ready sequence (spin up, aim)
		if (hasLauncher) {
			PairedLauncher launcher = mechanisms.get(PairedLauncher.class);
			launcher.openDynamic();
		}
		if (hasTransfer) {
			VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
			transfer.freeze();
		}
		if (hasIntake) {
			FlexVectorIntake intake = mechanisms.get(FlexVectorIntake.class);
			intake.crawl();
		}
		krakaTimer.resetTimer();
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// If no parts, we can't launch anything
		if (!hasTransfer || !hasLauncher || !hasIntake) {
			return true;
		}
		
		PairedLauncher launcher = mechanisms.get(PairedLauncher.class);
		VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
		
		// State machine for sequential ball firing
		switch (state) {
			case WAITING_TO_FIRE:
				// Only transition to FIRING if we are ready
				//if (launcher.isAtSpeed()) {
					state = State.FIRING;
					krakaTimer.resetTimer();
				//}
				break;
			
			case FIRING:
				if (krakaTimer.getElapsedTime() > KRAKATOA_TIME_MS) {
					transfer.freeze();
					state = State.COMPLETE;
				} else {
					transfer.advance();
				}
				
				break;
			
			case COMPLETE:
				// All done
				break;
		}
		
		if (state == State.COMPLETE) {
			launcher.close();
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
				launcher.lock();
			}
		}
		// Otherwise, launcher continues running for next action
	}
	
	@Override
	public String getName() {
		return "KRAKATOA";
	}
	
	private enum State {
		WAITING_TO_FIRE, // Ball at kicker, waiting for cooldown
		FIRING, // Currently firing
		COMPLETE // All balls fired
	}
}
