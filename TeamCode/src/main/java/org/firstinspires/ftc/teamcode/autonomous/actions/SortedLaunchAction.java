package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.LAUNCH_DEBOUNCE_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.SEARCH_TIMEOUT_MS;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.hardware.BallSwap;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.PairedLauncher;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.ColorUnifier;
import org.firstinspires.ftc.teamcode.software.game.Artifact;
import org.firstinspires.ftc.teamcode.software.game.Classifier;
import org.firstinspires.ftc.teamcode.software.game.Motif;

/**
 * Launches all balls in motif order using real-time color detection at the swap
 * position.
 * <p>
 * Uses the ColorUnifier to detect what ball is currently at the swap/launch
 * position.
 * The BallSwap mechanism buffers one ball when needed to achieve the correct
 * sequence.
 * <p>
 * State machine flow:
 * <ol>
 * <li>SEARCHING: Advance transfer until a ball is detected or timeout (1
 * second)</li>
 * <li>DECIDE: Check if current ball matches desired color</li>
 * <li>OPEN_GATE: Open launcher gate, wait for it to open</li>
 * <li>FIRING: Advance transfer to launch ball, wait for debounce</li>
 * <li>STORING: Store current ball in swap, go back to searching</li>
 * <li>RETRIEVING: Reverse transfer, grab ball from swap, go to decide</li>
 * <li>COMPLETE: All balls launched or timeout</li>
 * </ol>
 */
public class SortedLaunchAction implements AutonomousAction {
	private VerticalWheelTransfer transfer;
	private PairedLauncher launcher;
	private BallSwap swap;
	private FlexVectorIntake intake;
	private ColorUnifier colorUnifier;

	private Timer timer;
	private State state;
	private Artifact.Color currentBallColor;
	private Artifact.Color swappedBallColor;

	@Override
	public void initialize(MechanismManager mechanisms) {
		transfer = mechanisms.get(VerticalWheelTransfer.class);
		launcher = mechanisms.get(PairedLauncher.class);
		swap = mechanisms.get(BallSwap.class);
		intake = mechanisms.get(FlexVectorIntake.class);
		
		if (intake != null) {
			colorUnifier = intake.colorUnifier;
		}

		timer = new Timer();
		state = State.SEARCHING;
		currentBallColor = Artifact.Color.NONE;
		swappedBallColor = Artifact.Color.NONE;
		
		if (launcher != null) {
			launcher.ready();
			launcher.close();
		}
		if (transfer != null) {
			transfer.freeze();
		}
		if (intake != null) {
			intake.crawl();
		}
		if (swap != null) {
			swap.hold();
		}
		
		// Start searching
		timer.resetTimer();
		if (transfer != null) {
			transfer.advance();
		}
	}

	@Override
	public boolean execute(MechanismManager mechanisms) {
		if (transfer == null || launcher == null || colorUnifier == null) {
			return true;
		}
		
		// Keep launcher spun/aimed during the sequence
		launcher.ready();

		switch (state) {
			case SEARCHING:
				handleSearching();
				break;

			case DECIDE:
				handleDecide();
				break;
			
			case OPEN_GATE:
				handleOpenGate();
				break;
			
			case FIRING:
				handleFiring();
				break;
			
			case STORING:
				handleStoring();
				break;
			
			case REVERSING:
				handleReversing();
				break;
			
			case RETRIEVING:
				handleRetrieving();
				break;

			case COMPLETE:
			default:
				return true;
		}

		return state == State.COMPLETE;
	}
	
	private void handleSearching() {
		// Check for ball detection
		Artifact detected = colorUnifier.find();
		if (detected.color != Artifact.Color.NONE) {
			currentBallColor = detected.color;
			transfer.freeze();
			state = State.DECIDE;
			return;
		}
		
		// Keep advancing if not busy
		if (!transfer.isBusy()) {
			transfer.advance();
		}
		
		// Timeout: no ball found after SEARCH_TIMEOUT_MS
		if (timer.getElapsedTime() > SEARCH_TIMEOUT_MS) {
			// Check if swap has a ball we should retrieve
			if (swappedBallColor != Artifact.Color.NONE) {
				startReverse();
				state = State.REVERSING;
				return;
			}
			// Otherwise we're done
			finishLaunch();
			state = State.COMPLETE;
		}
	}
	
	private void handleDecide() {
		Artifact.Color desired = desiredColor();
		
		// If no preference or color matches, fire it
		if (desired == Artifact.Color.NONE || currentBallColor == desired) {
			startFire();
			return;
		}
		
		// If swap has the desired color, retrieve it (store current first if swap
		// empty)
		if (swappedBallColor == desired) {
			// Store current ball then retrieve
			startStore();
			return;
		}
		
		// If swap is empty, store current ball and search for next
		if (swappedBallColor == Artifact.Color.NONE) {
			startStore();
			return;
		}
		
		// Both current and swap have wrong colors - fire current anyway
		startFire();
	}
	
	private void handleOpenGate() {
		launcher.open();
		if (!launcher.isBusy()) {
			transfer.advance();
			timer.resetTimer();
			state = State.FIRING;
		}
	}
	
	private void handleFiring() {
		if (!transfer.isBusy() && timer.getElapsedTime() > LAUNCH_DEBOUNCE_TIME_MS) {
			// Record the shot
			addShotToClassifier(currentBallColor);
			currentBallColor = Artifact.Color.NONE;
			launcher.close();
			
			// Search for next ball
			timer.resetTimer();
			transfer.advance();
			state = State.SEARCHING;
		}
	}
	
	private void handleStoring() {
		// Wait for swap mechanism to finish
		if (swap != null && !swap.isBusy()) {
			// Ball is now stored in swap
			swappedBallColor = currentBallColor;
			currentBallColor = Artifact.Color.NONE;
			swap.hold();
			
			// Search for next ball
			timer.resetTimer();
			transfer.advance();
			state = State.SEARCHING;
		}
	}
	
	private void handleReversing() {
		// Wait for transfer to finish reversing
		if (!transfer.isBusy()) {
			if (swap != null) {
				swap.grab();
			}
			state = State.RETRIEVING;
		}
	}
	
	private void handleRetrieving() {
		// Wait for swap to finish grabbing
		if (swap != null && !swap.isBusy()) {
			// Ball is now at the launch position
			currentBallColor = swappedBallColor;
			swappedBallColor = Artifact.Color.NONE;
			state = State.DECIDE;
		}
	}
	
	private void startFire() {
		state = State.OPEN_GATE;
	}
	
	private void startStore() {
		if (swap != null) {
			swap.grab();
		}
		state = State.STORING;
	}
	
	private void startReverse() {
		transfer.reverse();
	}
	
	private void addShotToClassifier(Artifact.Color color) {
		if (color == Artifact.Color.NONE) {
			return;
		}
		MatchState.addArtifact(new Artifact(color, 0));
	}
	
	private Artifact.Color desiredColor() {
		Classifier classifier = MatchState.getClassifier();
		if (classifier == null) {
			return Artifact.Color.NONE;
		}
		Motif motif = classifier.getMotif();
		if (motif == null || motif == Motif.UNKNOWN || motif.state.length == 0) {
			return Artifact.Color.NONE;
		}
		int idx = classifier.getBallCount() % motif.state.length;
		return motif.state[idx].color;
	}
	
	private void finishLaunch() {
		if (launcher != null) {
			launcher.stop();
			launcher.close();
		}
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		if (interrupted && launcher != null) {
			launcher.stop();
			launcher.close();
		}
	}
	
	@Override
	public String getName() {
		return "Sorted Launch (" + state.name().toLowerCase() + ")";
	}
	
	private enum State {
		SEARCHING, // Advancing transfer looking for a ball
		DECIDE, // Deciding what to do with current ball
		OPEN_GATE, // Opening launcher gate
		FIRING, // Firing current ball
		STORING, // Storing current ball in swap
		REVERSING, // Reversing transfer to retrieve from swap
		RETRIEVING, // Grabbing ball from swap
		COMPLETE // All balls launched
	}
}
