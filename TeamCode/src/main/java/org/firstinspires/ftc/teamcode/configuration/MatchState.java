package org.firstinspires.ftc.teamcode.configuration;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRuntime;
import org.firstinspires.ftc.teamcode.software.game.Artifact;
import org.firstinspires.ftc.teamcode.software.game.Classifier;
import org.firstinspires.ftc.teamcode.software.game.Motif;

import java.util.ArrayList;
import java.util.List;

/**
 * A persistent static class that stores everything we know about the current
 * match
 * state.
 * <p>
 * This class replaces the OpMode.blackboard by acting as the static data
 * store itself, ensuring type safety and easy access from any OpMode without
 * needing to pass a HashMap.
 */
public class MatchState {

	// --- Private Static Storage ---
	// All match data is stored here. It persists between OpMode runs.
	
	/**
	 * Stores the robot's complete state: the detected Motif and all balls
	 * (Artifacts)
	 * currently held. Initialized to an empty state.
	 */
	private static Classifier classifier = Classifier.empty();
	
	/**
	 * Stores the selected alliance color. Defaults to BLUE.
	 */
	private static AllianceColor allianceColor = AllianceColor.BLUE;
	
	/**
	 * Stores the selected autonomous starting position. Defaults to FAR.
	 */
	private static AutoStartingPosition autoStartingPosition = AutoStartingPosition.FAR;
	
	/**
	 * Stores the selected autonomous runtime strategy. Defaults to DEFAULT.
	 */
	private static AutonomousRuntime autonomousRuntime = AutonomousRuntime.BEST;
	
	/**
	 * Stores the robot's last known pose, set at the end of Autonomous for
	 * TeleOp to use. Null if not set.
	 */
	private static Pose storedPose = null;
	
	/**
	 * Resets all static variables to their default state.
	 * Call this from your *very first* OpMode (e.g., a "INIT_Robot" OpMode)
	 * to ensure a clean state for every new match.
	 */
	public static void reset() {
		classifier = Classifier.empty();
		allianceColor = AllianceColor.BLUE;
		autoStartingPosition = AutoStartingPosition.FAR;
		autonomousRuntime = AutonomousRuntime.BEST;
		storedPose = null;
	}
	
	// --- Classifier & Artifact Methods ---
	
	public static Classifier getClassifier() {
		return classifier;
	}
	
	public static void setClassifier(Classifier newClassifier) {
		if (newClassifier != null) {
			classifier = newClassifier;
		}
	}
	
	/**
	 * Adds an artifact (ball) to the classifier's sequential storage.
	 *
	 * @param artifact The Artifact to add (e.g., Artifact.GREEN).
	 */
	public static void addArtifact(Artifact artifact) {
		if (classifier != null) {
			classifier.addBall(artifact);
		}
	}
	
	/**
	 * Empties all artifacts from the classifier, but keeps the Motif.
	 */
	public static void emptyClassifierBalls() {
		if (classifier != null) {
			// Re-create the classifier, preserving the motif but using an empty ball array
			classifier = new Classifier(classifier.getMotif(), new Artifact[0]);
		} else {
			classifier = Classifier.empty();
		}
	}
	
	/**
	 * Gets the color of the next artifact needed to score, based on the
	 * detected Motif and the number of balls already in the classifier.
	 *
	 * @return The Artifact.Color needed, or Artifact.Color.NONE if unknown.
	 */
	public static Artifact.Color nextArtifactNeeded() {
		if (classifier == null)
			return Artifact.Color.NONE;
		
		Motif motif = classifier.getMotif();
		// Return NONE if motif is unknown or has no pattern
		if (motif == null || motif == Motif.UNKNOWN || motif.state.length == 0) {
			return Artifact.Color.NONE;
		}
		
		Artifact[] motifPattern = motif.state;
		int currentBallCount = classifier.getBallCount();
		
		// Use modulo to loop back to the beginning of the motif sequence
		int index = currentBallCount % motifPattern.length;
		return motifPattern[index].color;
	}
	
	/**
	 * Gets the next three artifact colors needed, in sequence.
	 *
	 * @return A List of Artifact.Color, empty if motif is unknown.
	 */
	public static List<Artifact.Color> nextThreeArtifactsNeeded() {
		List<Artifact.Color> nextThree = new ArrayList<>();
		if (classifier == null)
			return nextThree;
		
		Motif motif = classifier.getMotif();
		// Return empty list if motif is unknown or has no pattern
		if (motif == null || motif == Motif.UNKNOWN || motif.state.length == 0) {
			return nextThree;
		}
		
		Artifact[] motifPattern = motif.state;
		int currentBallCount = classifier.getBallCount();
		
		// Loop three times to get the next three colors
		for (int i = 0; i < 3; i++) {
			// Use modulo to loop back to the beginning of the motif sequence
			int index = (currentBallCount + i) % motifPattern.length;
			nextThree.add(motifPattern[index].color);
		}
		
		return nextThree;
	}
	
	// --- Alliance & Position Methods ---
	
	public static AllianceColor getAllianceColor() {
		return allianceColor;
	}
	
	public static void setAllianceColor(AllianceColor color) {
		if (color != null) {
			MatchState.allianceColor = color;
		}
	}
	
	public static AutoStartingPosition getAutoStartingPosition() {
		return autoStartingPosition;
	}
	
	public static void setAutoStartingPosition(AutoStartingPosition position) {
		if (position != null) {
			MatchState.autoStartingPosition = position;
		}
	}
	
	public static AutonomousRuntime getAutonomousRuntime() {
		return autonomousRuntime;
	}
	
	public static void setAutonomousRuntime(AutonomousRuntime runtime) {
		if (runtime != null) {
			MatchState.autonomousRuntime = runtime;
		}
	}
	
	/**
	 * Cycles to the next autonomous runtime.
	 */
	public static void nextAutonomousRuntime() {
		autonomousRuntime = autonomousRuntime.next();
	}
	
	/**
	 * Cycles to the previous autonomous runtime.
	 */
	public static void previousAutonomousRuntime() {
		autonomousRuntime = autonomousRuntime.previous();
	}
	
	/**
	 * Cycles to the next autonomous runtime that supports the given position.
	 *
	 * @param position The starting position that must be supported
	 */
	public static void nextAutonomousRuntimeFor(AutoStartingPosition position) {
		autonomousRuntime = autonomousRuntime.nextFor(position);
	}
	
	/**
	 * Cycles to the previous autonomous runtime that supports the given position.
	 *
	 * @param position The starting position that must be supported
	 */
	public static void previousAutonomousRuntimeFor(AutoStartingPosition position) {
		autonomousRuntime = autonomousRuntime.previousFor(position);
	}
	
	/**
	 * Gets the Autonomous starting position for the robot based on the match
	 * settings.
	 * <p>
	 * Uses BLUE poses as reference and mirrors for RED alliance.
	 *
	 * @return A Pose of the starting position
	 */
	public static Pose getAutonomousStartingPose() {
		// Get BLUE pose (our reference)
		Pose bluePose = autoStartingPosition == AutoStartingPosition.CLOSE
				? Settings.Positions.AutoStart.CLOSE
				: Settings.Positions.AutoStart.FAR;
		
		// Mirror for RED alliance
		if (allianceColor == AllianceColor.RED) {
			return Settings.Field.mirrorPose(bluePose);
		} else {
			return bluePose;
		}
	}
	
	/**
	 * Gets the TeleOp starting position for the robot based on the actual stored
	 * pose
	 * from the previous OpMode (typically Autonomous).
	 * <p>
	 * If no actual pose was stored, falls back to the default reset position.
	 *
	 * @return A Pose of the starting position
	 */
	public static Pose getTeleOpStartingPose() {
		// First, try to get the actual stored pose
		if (storedPose != null) {
			return storedPose;
		}
		
		// Fall back to predefined poses if actual pose is unavailable
		// Get BLUE pose (our reference)
		Pose bluePose = Settings.Positions.Reset.HUMAN_PLAYER_ZONE;
		
		// Mirror for RED alliance
		if (allianceColor == AllianceColor.RED) {
			return Settings.Field.mirrorPose(bluePose);
		} else {
			return bluePose;
		}
	}
	
	// --- Stored Pose Methods ---
	
	/**
	 * Retrieves the stored robot pose from the previous OpMode.
	 *
	 * @return The stored pose, or null if no pose was stored
	 */
	public static Pose getStoredPose() {
		return storedPose;
	}
	
	/**
	 * Stores the actual robot pose for use by subsequent OpModes.
	 * This pose will be used as the starting position for the next OpMode.
	 *
	 * @param pose The actual robot pose to store
	 */
	public static void setStoredPose(Pose pose) {
		MatchState.storedPose = pose;
	}
	
	/**
	 * Clears the stored pose.
	 */
	public static void clearStoredPose() {
		storedPose = null;
	}
	
	// --- Enums ---
	
	public enum AllianceColor {
		RED,
		BLUE,
		UNKNOWN
	}
	
	public enum AutoStartingPosition {
		CLOSE,
		FAR
	}
}
