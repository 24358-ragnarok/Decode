package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.software.game.Artifact;
import org.firstinspires.ftc.teamcode.software.game.Classifier;
import org.firstinspires.ftc.teamcode.software.game.Motif;

import java.util.ArrayList;
import java.util.Comparator;

/**
 * Interface between the Limelight camera and the robot, providing vision processing
 * capabilities for artifact detection, AprilTag recognition, and goal targeting.
 * <p>
 * Manages pipeline switching between different vision modes:
 * - AprilTag detection for motif recognition and goal targeting
 * - Green artifact detection
 * - Purple artifact detection
 * <p>
 * Optimized to minimize overhead by avoiding unnecessary pipeline switches and
 * providing efficient result caching.
 */
public class LimelightManager extends Mechanism {
	private static final Comparator<ColoredTarget> TX_COMPARATOR = Comparator.comparingDouble(t -> t.tx);
	public final Limelight3A limelight;
	LLResult currentResult;
	/**
	 * Current active pipeline - starts with AprilTag to detect obelisk during auto start
	 */
	Pipeline currentPipeline = Pipeline.APRILTAG;
	
	public LimelightManager(Limelight3A limelight) {
		this.limelight = limelight;
		start(); // limelight is a non-physical system and thus can be initialized at any time
	}
	
	/**
	 * Initializes limelight with polling rate of 100 Hz.
	 * Sets the current pipeline and starts the camera streaming.
	 */
	public void start() {
		setCurrentPipeline(currentPipeline);
		limelight.start();
		limelight.setPollRateHz(100);
	}
	
	/**
	 * Updates the limelight result cache.
	 * Note: This causes significant overhead and should be avoided in tight loops.
	 */
	public void update() {
		limelight.getLatestResult();
	}
	
	public void stop() {
		limelight.stop();
	}
	
	/**
	 * Updates the data and checks if there is a desired object detected
	 *
	 * @param color The Artifact.Color (GREEN or PURPLE) to detect.
	 * @return if an artifact is detected
	 */
	public boolean detectArtifact(Artifact.Color color) {
		setCurrentPipeline(getPipelineFromColor(color));
		currentResult = limelight.getLatestResult();
		return currentResult.getTx() != 0 && currentResult.getTy() != 0
				&& (Math.abs(currentResult.getTx()) < Settings.Vision.LL_WINDOW_SIZE_DEGREES);
	}
	
	/**
	 * Helper to get the correct pipeline from an Artifact.Color.
	 *
	 * @param color The Artifact.Color
	 * @return The corresponding Pipeline enum.
	 */
	public final Pipeline getPipelineFromColor(Artifact.Color color) {
		if (color == Artifact.Color.GREEN) {
			return Pipeline.GREEN;
		} else {
			return Pipeline.PURPLE;
		}
	}
	
	public Pipeline getCurrentPipeline() {
		return currentPipeline;
	}
	
	/**
	 * Switches the current pipeline to a new pipeline, optimized to avoid redundant switches.
	 *
	 * @param newPipeline The new pipeline to switch to:
	 *                    APRILTAG (1), GREEN (2), PURPLE (3)
	 */
	public void setCurrentPipeline(Pipeline newPipeline) {
		if (newPipeline == currentPipeline) {
			return;
		}
		currentPipeline = newPipeline;
		// Limelight pipelines are 1-indexed
		limelight.pipelineSwitch(currentPipeline.ordinal() + 1);
	}
	
	/**
	 * Attempts to detect the unique three-artifact motif using AprilTag detection.
	 *
	 * @return The detected Motif object (GPP, PGP, or PPG), or Motif.UNKNOWN if no valid tag is found.
	 */
	public Motif detectMotif() {
		setCurrentPipeline(Pipeline.APRILTAG);
		LLResult result = limelight.getLatestResult();
		
		for (LLResultTypes.FiducialResult r : result.getFiducialResults()) {
			Motif detected = Motif.fromApriltag(r.getFiducialId());
			// If fromApriltag returns a known motif (not UNKNOWN), return it immediately.
			if (detected != Motif.UNKNOWN) {
				return detected;
			}
		}
		// If no valid motif tags were found after checking all results
		return Motif.UNKNOWN;
	}
	
	/**
	 * Detects all visible Green and Purple balls, sorts them by their horizontal position (tx),
	 * and combines them with the pre-detected Motif to create the robot's Classifier state.
	 *
	 * @param classifierMotif The Motif (GPP, PGP, PPG) that was *already detected* in a previous step.
	 * @return A new Classifier object containing the Motif and the spatially-sorted Artifacts.
	 */
	public Classifier coerceClassifierState(Motif classifierMotif) {
		ArrayList<ColoredTarget> detectedBalls = new ArrayList<>();
		
		// 1. Detect all PURPLE balls
		setCurrentPipeline(Pipeline.PURPLE);
		currentResult = limelight.getLatestResult();
		if (currentResult.getColorResults() != null) {
			for (LLResultTypes.ColorResult t : currentResult.getColorResults()) {
				detectedBalls.add(new ColoredTarget(Artifact.Color.PURPLE, t.getTargetXPixels()));
			}
		}
		
		// 2. Detect all GREEN balls
		setCurrentPipeline(Pipeline.GREEN);
		currentResult = limelight.getLatestResult();
		if (currentResult.getColorResults() != null) {
			for (LLResultTypes.ColorResult t : currentResult.getColorResults()) {
				detectedBalls.add(new ColoredTarget(Artifact.Color.GREEN, t.getTargetXPixels()));
			}
		}
		
		// 3. Sort the combined list by the horizontal offset (tx)
		detectedBalls.sort(TX_COMPARATOR);
		
		// 4. Convert the sorted list into an Artifact[] array, enforcing 9-ball limit
		// We use the static Artifact instances (GREEN/PURPLE) which have ticks=0,
		// as this is just for pattern/color sequence recognition.
		final int MAX_CLASSIFIER_CAPACITY = 9;
		int numBallsToCopy = Math.min(detectedBalls.size(), MAX_CLASSIFIER_CAPACITY);
		
		Artifact[] sortedArtifacts = new Artifact[numBallsToCopy];
		for (int i = 0; i < numBallsToCopy; i++) {
			ColoredTarget ct = detectedBalls.get(i);
			sortedArtifacts[i] = (ct.color == Artifact.Color.GREEN) ? Artifact.GREEN : Artifact.PURPLE;
		}
		
		// 5. Create and return the new Classifier
		// The Classifier's constructor will handle this array (up to 9 elements).
		return new Classifier(classifierMotif, sortedArtifacts);
	}
	
	/**
	 * Gets the latest Limelight result for goal targeting.
	 * Assumes the goal is an AprilTag.
	 *
	 * @return The latest LLResult.
	 */
	public LLResult detectGoal() {
		setCurrentPipeline(Pipeline.APRILTAG);
		return limelight.getLatestResult();
	}
	
	public enum Pipeline {
		APRILTAG, // pipe 1
		PURPLE,    // pipe 2
		GREEN,   // pipe 3
		UNKNOWN
	}
	
	/**
	 * A helper class to temporarily store detected balls with their color and x-position
	 * for sorting before creating the final Artifact array.
	 */
	private static class ColoredTarget {
		public Artifact.Color color;
		public double tx; // Horizontal offset
		
		public ColoredTarget(Artifact.Color color, double tx) {
			this.color = color;
			this.tx = tx;
		}
	}
}