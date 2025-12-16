package org.firstinspires.ftc.teamcode.software.game;

import androidx.annotation.NonNull;

import java.util.Arrays;

/**
 * Stores the detected Motif and manages the sequential queue of up to 9 Artifacts (balls)
 * currently held by the robot for scoring.
 */
public class Classifier {
	private static final int MAX_CAPACITY = 9;
	private final Motif motif;
	private final Artifact[] state; // Fixed size of 9 balls max
	private int ballCount;          // Tracks the number of balls currently stored (and the next available index)
	
	/**
	 * Initializes the Classifier with a determined Motif and an initial set of Artifacts.
	 *
	 * @param classifierMotif The determined Motif pattern (GPP, PGP, PPG, or UNKNOWN).
	 * @param initial         An array of Artifacts to pre-load into the state.
	 */
	public Classifier(Motif classifierMotif, Artifact[] initial) {
		this.motif = classifierMotif;
		this.state = new Artifact[MAX_CAPACITY];
		this.ballCount = 0;
		
		// 1. Copy initial artifacts, respecting the MAX_CAPACITY limit
		if (initial != null) {
			int toCopy = Math.min(initial.length, MAX_CAPACITY);
			// Ensure we copy the actual object, not just the reference (if necessary)
			// For simplicity here, we assume the provided artifacts are ready to be stored.
			System.arraycopy(initial, 0, this.state, 0, toCopy);
			this.ballCount = toCopy;
		}
		
		// 2. Fill the rest of the fixed array with Artifact.NONE to prevent null pointers
		for (int i = this.ballCount; i < MAX_CAPACITY; i++) {
			this.state[i] = Artifact.NONE;
		}
	}
	
	/**
	 * build a classifier with an known motif and default empty state
	 **/
	public Classifier(Motif classifierMotif) {
		this(classifierMotif, new Artifact[0]);
	}
	
	/**
	 * Factory method to create an empty Classifier for initial use.
	 * Initializes with an UNKNOWN motif and zero balls stored.
	 *
	 * @return A new, empty Classifier instance.
	 */
	public static Classifier empty() {
		// Create a temporary 9-element array initialized to NONE
		Artifact[] emptyState = new Artifact[MAX_CAPACITY];
		Arrays.fill(emptyState, Artifact.NONE);
		
		// Pass the unknown motif and the empty state to the constructor
		Classifier emptyClassifier = new Classifier(Motif.UNKNOWN, emptyState);
		
		// Critically, set the ballCount to 0 to indicate no balls are *active*
		emptyClassifier.ballCount = 0;
		return emptyClassifier;
	}
	
	/**
	 * Adds an artifact (ball) to the end of the sequence.
	 * The ball is placed at the index equal to the current ballCount.
	 *
	 * @param ball The Artifact to add (e.g., Artifact.GREEN or Artifact.PURPLE).
	 * @return true if the ball was successfully added, false if the capacity is full.
	 */
	public boolean addBall(Artifact ball) {
		if (ballCount < MAX_CAPACITY) {
			// Add the ball at the current count index
			this.state[ballCount] = ball;
			// Increment the count to point to the next available slot
			ballCount++;
			return true;
		}
		return false; // Storage is full
	}
	
	// --- Getters ---
	
	public Motif getMotif() {
		return motif;
	}
	
	/**
	 * Gets the current array of stored artifacts (balls).
	 *
	 * @return The full 9-element array. Check getBallCount() to know how many are active.
	 */
	public Artifact[] getState() {
		// Returning a defensive copy is usually best practice, but for FTC
		// simplicity, we return the reference.
		return state;
	}
	
	public int getBallCount() {
		return ballCount;
	}
	
	@NonNull
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("Classifier Motif: ").append(motif.toString()).append("\n");
		sb.append("Stored Balls (").append(ballCount).append("/").append(MAX_CAPACITY).append("): [");
		
		for (int i = 0; i < ballCount; i++) {
			sb.append(state[i].color.name());
			if (i < ballCount - 1) {
				sb.append(", ");
			}
		}
		sb.append("]");
		return sb.toString();
	}
}