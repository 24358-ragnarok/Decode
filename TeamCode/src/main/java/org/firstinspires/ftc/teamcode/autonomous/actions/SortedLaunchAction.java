package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.LAUNCH_DEBOUNCE_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.FIRING_POSITION_TICKS;

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
import org.firstinspires.ftc.teamcode.software.game.Motif;

/**
 * Launches all balls in motif order using the BallSwap buffer when needed.
 * Falls back to launch-anyway if motif is unknown or impossible to satisfy.
 */
public class SortedLaunchAction implements AutonomousAction {
	private boolean hasTransfer;
	private boolean hasLauncher;
	private boolean hasSwap;
	
	private VerticalWheelTransfer transfer;
	private PairedLauncher launcher;
	private BallSwap swap;
	
	private Timer timer;
	private State state;
	private Artifact firingArtifact;
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		transfer = mechanisms.get(VerticalWheelTransfer.class);
		launcher = mechanisms.get(PairedLauncher.class);
		swap = mechanisms.get(BallSwap.class);
		
		hasTransfer = transfer != null;
		hasLauncher = launcher != null;
		hasSwap = swap != null;
		
		timer = new Timer();
		state = State.ALIGNING;
		firingArtifact = null;
		
		// Preload: assume green held in swapper at start of auto if none stored
		// already.
		if (hasSwap && transfer != null && !swap.hasHeldArtifact()) {
			Artifact preload = new Artifact(Artifact.Color.GREEN, transfer.getTicks());
			preload.beingSwapped = true;
			swap.storeArtifact(preload);
		}
		
		if (hasLauncher) {
			launcher.ready();
			launcher.close();
		}
		if (hasTransfer) {
			transfer.freeze();
		}
		if (hasSwap) {
			swap.hold();
		}
		mechanisms.ifValid(mechanisms.get(FlexVectorIntake.class), FlexVectorIntake::crawl);
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		if (!hasTransfer || !hasLauncher) {
			return true;
		}
		
		// Keep launcher spun/aimed during the sequence.
		launcher.ready();
		
		switch (state) {
			case ALIGNING:
				if (transfer.isEmpty() && getSwapArtifact() == null) {
					finishLaunch();
					state = State.COMPLETE;
					return true;
				}
				// await previous actions
				if (!transfer.isBusy() && !launcher.isBusy() && !swap.isBusy()) {
					launcher.close();
					transfer.advance();
					state = State.WAIT_ALIGN;
				}
				break;
			
			case WAIT_ALIGN:
				if (!transfer.isBusy() && !launcher.isBusy()) {
					state = State.DECIDE;
				}
				break;
			
			case DECIDE:
				Artifact.Color desired = desiredColor();
				Artifact top = topArtifact();
				
				if (top == null) {
					if (getSwapArtifact() != null) {
						insertSwapAsTop();
						state = State.ALIGNING;
						break;
					}
					finishLaunch();
					state = State.COMPLETE;
					return true;
				}
				
				if (desired == Artifact.Color.NONE || top.color == desired) {
					beginFire(top);
					break;
				}
				
				// If swap holds the desired color, reinsert it and retry.
				Artifact swapped = getSwapArtifact();
				if (swapped != null && swapped.color == desired) {
					insertSwapAsTop();
					state = State.ALIGNING;
					break;
				}
				
				// If desired color exists deeper and swap is free, buffer the top ball.
				if (swapped == null && containsColor(desired)) {
					bufferTop(top);
					state = State.ALIGNING;
					break;
				}
				
				// Impossible or unknown scenario; launch anyway.
				beginFire(top);
				break;
			
			case WAIT_FIRE:
				if (!transfer.isBusy() && timer.getElapsedTime() > LAUNCH_DEBOUNCE_TIME_MS) {
					removeFiredArtifact();
					launcher.close();
					state = State.ALIGNING;
				}
				break;
			
			case COMPLETE:
			default:
				return true;
		}
		
		return state == State.COMPLETE;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		if (interrupted && hasLauncher) {
			launcher.stop();
			launcher.close();
		}
	}
	
	@Override
	public String getName() {
		return "Sorted Launch (" + state.name().toLowerCase() + ")";
	}
	
	private void beginFire(Artifact top) {
		firingArtifact = top;
		launcher.open();
		transfer.advance();
		timer.resetTimer();
		state = State.WAIT_FIRE;
	}
	
	private void bufferTop(Artifact top) {
		if (!hasSwap)
			return;
		swap.grab();
		top.beingSwapped = true;
		swap.storeArtifact(top);
		removeArtifact(top);
	}
	
	private void insertSwapAsTop() {
		if (!hasSwap)
			return;
		// Move swap into grabbing position to place the held ball back.
		swap.grab();
		Artifact held = swap.takeHeldArtifact();
		if (held == null || held.color == Artifact.Color.NONE) {
			return;
		}
		// Place the swapped artifact at the firing position based on current ticks.
		held.transferTicksWhenAtEntrance = transfer.getTicks() - FIRING_POSITION_TICKS;
		held.beingSwapped = false;
		insertArtifact(held);
	}
	
	private Artifact topArtifact() {
		double greatestTicks = Double.NEGATIVE_INFINITY;
		Artifact top = null;
		for (Artifact a : transfer.artifacts) {
			if (a == null || a.color == Artifact.Color.NONE)
				continue;
			double ticksTraveled = transfer.getTicks() - a.transferTicksWhenAtEntrance;
			if (Math.abs(ticksTraveled) > greatestTicks) {
				greatestTicks = Math.abs(ticksTraveled);
				top = a;
			}
		}
		return top;
	}
	
	private boolean containsColor(Artifact.Color color) {
		for (Artifact a : transfer.artifacts) {
			if (a != null && a.color == color) {
				return true;
			}
		}
		Artifact swapped = getSwapArtifact();
		return swapped != null && swapped.color == color;
	}
	
	private void removeArtifact(Artifact target) {
		for (int i = 0; i < transfer.artifacts.length; i++) {
			Artifact a = transfer.artifacts[i];
			if (a == target) {
				transfer.artifacts[i] = Artifact.NONE;
				return;
			}
		}
	}
	
	private void insertArtifact(Artifact artifact) {
		for (int i = 0; i < transfer.artifacts.length; i++) {
			if (transfer.artifacts[i].color == Artifact.Color.NONE) {
				transfer.artifacts[i] = artifact;
				return;
			}
		}
		// If no space, overwrite the least advanced artifact.
		int idx = 0;
		double smallestTicks = Double.POSITIVE_INFINITY;
		for (int i = 0; i < transfer.artifacts.length; i++) {
			Artifact a = transfer.artifacts[i];
			double ticksTraveled = transfer.getTicks() - a.transferTicksWhenAtEntrance;
			if (ticksTraveled < smallestTicks) {
				smallestTicks = ticksTraveled;
				idx = i;
			}
		}
		transfer.artifacts[idx] = artifact;
	}
	
	private void removeFiredArtifact() {
		if (firingArtifact == null) {
			return;
		}
		removeArtifact(firingArtifact);
		addShotToClassifier();
		firingArtifact = null;
	}
	
	private void addShotToClassifier() {
		if (firingArtifact == null) {
			return;
		}
		MatchState.addArtifact(new Artifact(firingArtifact.color, 0));
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
		if (hasLauncher) {
			launcher.stop();
			launcher.close();
		}
	}
	
	private Artifact getSwapArtifact() {
		if (!hasSwap) {
			return null;
		}
		Artifact a = swap.peekHeldArtifact();
		if (a == null || a.color == Artifact.Color.NONE) {
			return null;
		}
		return a;
	}
	
	private enum State {
		ALIGNING,
		WAIT_ALIGN,
		DECIDE,
		WAIT_FIRE,
		COMPLETE
	}
}
