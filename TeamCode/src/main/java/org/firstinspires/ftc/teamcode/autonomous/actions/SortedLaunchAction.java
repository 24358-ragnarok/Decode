package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.LAUNCH_EXIT_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.SEARCH_TIMEOUT_MS;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.hardware.BallSwap;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.PairedLauncher;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.game.Artifact;
import org.firstinspires.ftc.teamcode.software.game.Classifier;

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
	
	private Timer timer;
	private State state;
	private Artifact.Color currentBallColor;
	private boolean mayStore = false;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		transfer = mechanisms.get(VerticalWheelTransfer.class);
		launcher = mechanisms.get(PairedLauncher.class);
		swap = mechanisms.get(BallSwap.class);
		intake = mechanisms.get(FlexVectorIntake.class);
		
		timer = new Timer();
		state = State.SEARCHING;
		currentBallColor = Artifact.Color.NONE;
		mayStore = false;
		
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
			swap.moveToHold();
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		if (transfer == null || launcher == null || intake == null) {
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
				continueStoring();
				break;
			
			case RETRIEVING:
				continueRetrieve();
				break;
			
			case COMPLETE:
			default:
				return true;
		}
		
		return state == State.COMPLETE;
	}
	
	private void handleSearching() {
		// Check for ball detection
		Artifact detected = transfer.detect();
		
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
			if (swap.hasHeldArtifact()) {
				swap.moveToTransfer();
				state = State.RETRIEVING;
				return;
			}
			// Otherwise we're done
			finishLaunch();
			state = State.COMPLETE;
		}
	}
	
	private void handleDecide() {
		Artifact.Color desired = desiredColor();
		
		if (swap.hasHeldArtifact()) {
			Artifact.Color swappedColor = swap.peekHeldArtifact().color;
			
			// If swapping is the only way to match, do so
			if (swappedColor == desired && currentBallColor != desired) {
				// Start swapping in
				startRetrieve();
				return;
			}
		} else {
			// if we can store and look for color, do so
			if (desired != Artifact.Color.NONE && currentBallColor != desired && mayStore) {
				// Store current ball then retrieve
				startStore();
				return;
			}
		}
		
		// Swap has not triggered any action; we are good to fire
		startFire();
	}
	
	private void handleOpenGate() {
		if (!launcher.isBusy()) {
			transfer.advance();
			timer.resetTimer();
			state = State.FIRING;
		}
	}
	
	private void handleFiring() {
		if (!transfer.isBusy() && timer.getElapsedTime() > LAUNCH_EXIT_TIME_MS) {
			// Record the shot
			addShotToClassifier(currentBallColor);
			currentBallColor = Artifact.Color.NONE;
			launcher.close();
			
			// Search for next ball
			timer.resetTimer();
			transfer.advance();
			mayStore = true;
			state = State.SEARCHING;
		}
	}
	
	private void continueStoring() {
		// Wait for swap mechanism to finish
		if (!swap.isBusy()) {
			// Ball is now stored in swap
			swap.storeArtifact(new Artifact(currentBallColor, 0, true));
			currentBallColor = Artifact.Color.NONE;
			
			// Search for next ball
			timer.resetTimer();
			transfer.advance();
			state = State.SEARCHING;
		}
	}
	
	private void startRetrieve() {
		transfer.reverse();
		currentBallColor = Artifact.Color.NONE;
		state = State.RETRIEVING;
	}
	
	private void continueRetrieve() {
		if (!transfer.isBusy() && currentBallColor == Artifact.Color.NONE) {
			currentBallColor = swap.takeHeldArtifact().color;
			swap.moveToTransfer();
		}
		if (currentBallColor != Artifact.Color.NONE && !swap.isBusy()) {
			mayStore = false;
			state = State.DECIDE;
		}
	}
	
	private void startFire() {
		launcher.open();
		state = State.OPEN_GATE;
	}
	
	private void startStore() {
		swap.moveToHold();
		state = State.STORING;
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
		return classifier.getNextDesiredColor();
	}
	
	private void finishLaunch() {
		if (launcher != null) {
			launcher.stop();
			swap.moveToHold();
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
		return state.toString() + ", " + currentBallColor + " seen and " + swap.peekHeldArtifact().color + " stored";
	}
	
	private enum State {
		SEARCHING, // Advancing transfer looking for a ball
		DECIDE, // Deciding what to do with current ball
		OPEN_GATE, // Opening launcher gate
		FIRING, // Firing current ball
		STORING, // Storing current ball in swap
		RETRIEVING, // Grabbing ball from swap
		COMPLETE // All balls launched
	}
}
