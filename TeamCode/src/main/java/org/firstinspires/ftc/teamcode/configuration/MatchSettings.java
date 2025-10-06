package org.firstinspires.ftc.teamcode.configuration;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

/**
 * A persistent object that stores everything we know about the current match
 * state.
 *
 * @noinspection unchecked, we know what the blackboard is storing
 */
public class MatchSettings {
	private static final String CLASSIFIER_STATE_KEY = "classifierState";
	private static final String MOTIF_KEY = "motif";
	private static final String AUTO_KEY = "autoStartingPosition";
	private static final String ALLIANCE_COLOR_KEY = "allianceColor";
	private final HashMap<String, Object> blackboard;
	
	public MatchSettings(HashMap<String, Object> blackboard) {
		this.blackboard = blackboard;
	}
	
	/**
	 * Returns an array of ArtifactColor representing the sequence for the given
	 * Motif.
	 * The order of colors corresponds to the Motif's letter order.
	 * <p>
	 * Example:
	 * Motif.PPG -> [PURPLE, PURPLE, GREEN]
	 * Motif.PGP -> [PURPLE, GREEN, PURPLE]
	 * Motif.GPP -> [GREEN, PURPLE, PURPLE]
	 *
	 * @param motif The motif to convert
	 * @return An array of ArtifactColor in motif order
	 */
	public static ArtifactColor[] motifToArtifactColors(Motif motif) {
		if (motif == null)
			return null;
		String motifStr = motif.name();
		ArtifactColor[] colors = new ArtifactColor[3];
		for (int i = 0; i < 3; i++) {
			char c = motifStr.charAt(i);
			switch (c) {
				case 'P':
					colors[i] = ArtifactColor.PURPLE;
					break;
				case 'G':
					colors[i] = ArtifactColor.GREEN;
					break;
				default:
					throw new IllegalArgumentException("Unknown motif character: " + c);
			}
		}
		return colors;
	}
	
	public AllianceColor getAllianceColor() {
		String color = (String) blackboard.get(ALLIANCE_COLOR_KEY);
		return Objects.equals(color, "red") ? AllianceColor.RED : AllianceColor.BLUE;
	}
	
	public void setAllianceColor(AllianceColor color) {
		if (color != null) {
			blackboard.put(ALLIANCE_COLOR_KEY, color.name().toLowerCase());
		}
	}
	
	/**
	 * Gets the Autonomous starting position for the robot based on the match
	 * settings.
	 * <p>
	 * Uses BLUE poses as reference and mirrors for RED alliance.
	 *
	 * @return A Pose of the starting position
	 */
	public Pose getAutonomousStartingPose() {
		AllianceColor allianceColor = getAllianceColor();
		AutoStartingPosition startingPosition = getAutoStartingPosition();
		
		// Get BLUE pose (our reference)
		Pose bluePose = startingPosition == AutoStartingPosition.CLOSE
				? Settings.Autonomous.BlueClose.START
				: Settings.Autonomous.BlueFar.START;
		
		// Mirror for RED alliance
		if (allianceColor == AllianceColor.RED) {
			return mirrorPose(bluePose);
		} else {
			return bluePose;
		}
	}
	
	/**
	 * Gets the TeleOp starting position for the robot based on the end of the
	 * Autonomous period.
	 * <p>
	 * Uses BLUE poses as reference and mirrors for RED alliance.
	 *
	 * @return A Pose of the starting position
	 */
	public Pose getTeleOpStartingPose() {
		// If either of these values are unset, Auto has not yet been run,
		// so we have nothing to base starting position on. Instead, assume the robot is
		// on the center of the field,
		// our designated resetting position during testing.
		if (blackboard.get(ALLIANCE_COLOR_KEY) == null || blackboard.get(AUTO_KEY) == null) {
			return Settings.Field.RESET_POSE;
		}
		
		AllianceColor allianceColor = getAllianceColor();
		AutoStartingPosition startingPosition = getAutoStartingPosition();
		
		// Get BLUE pose (our reference)
		Pose bluePose = startingPosition == AutoStartingPosition.CLOSE
				? Settings.Autonomous.BlueClose.PARK
				: Settings.Autonomous.BlueFar.PARK;
		
		// Mirror for RED alliance
		if (allianceColor == AllianceColor.RED) {
			return mirrorPose(bluePose);
		} else {
			return bluePose;
		}
	}
	
	public AutoStartingPosition getAutoStartingPosition() {
		String position = (String) blackboard.get(AUTO_KEY);
		return Objects.equals(position, "close") ? AutoStartingPosition.CLOSE : AutoStartingPosition.FAR;
	}
	
	public void setAutoStartingPosition(AutoStartingPosition position) {
		if (position != null) {
			blackboard.put(AUTO_KEY, position.name().toLowerCase());
		}
	}
	
	public Motif getMotif() {
		String motif = (String) blackboard.get(MOTIF_KEY);
		if (motif != null) {
			return Motif.valueOf(motif.toUpperCase());
		} else {
			return Motif.UNKNOWN;
		}
	}
	
	public void setMotif(Motif motif) {
		if (motif != null) {
			blackboard.put(MOTIF_KEY, motif.name().toLowerCase());
		}
	}
	
	// Get current state
	public List<ArtifactColor> getClassifierState() {
		return (List<ArtifactColor>) blackboard.getOrDefault(CLASSIFIER_STATE_KEY, new ArrayList<ArtifactColor>());
	}
	
	// Set state
	public void setClassifierState(List<ArtifactColor> state) {
		List<ArtifactColor> copy = new ArrayList<>(state.size() > 9 ? state.subList(0, 9) : state);
		blackboard.put(CLASSIFIER_STATE_KEY, copy);
	}
	
	// Append artifact
	public void addArtifact(ArtifactColor artifact) {
		List<ArtifactColor> state = getClassifierState();
		if (state.size() >= 9)
			return;
		state.add(artifact);
		blackboard.put(CLASSIFIER_STATE_KEY, state);
	}
	
	public void emptyClassifier() {
		blackboard.put(CLASSIFIER_STATE_KEY, new ArrayList<ArtifactColor>());
	}
	
	public void incrementClassifier() {
		List<ArtifactColor> state = getClassifierState();
		if (state.size() >= 9)
			return;
		state.add(ArtifactColor.UNKNOWN);
		blackboard.put(CLASSIFIER_STATE_KEY, state);
	}
	
	public ArtifactColor nextArtifactNeeded() {
		Motif motif = getMotif();
		if (motif == null || motif == Motif.UNKNOWN)
			return ArtifactColor.UNKNOWN;
		
		List<ArtifactColor> state = getClassifierState();
		ArtifactColor[] motifColors = motifToArtifactColors(motif);
		if (motifColors == null || motifColors.length == 0)
			return ArtifactColor.UNKNOWN;
		
		int index = state.size() % motifColors.length;
		return motifColors[index];
	}
	
	public List<ArtifactColor> nextThreeArtifactsNeeded() {
		Motif motif = getMotif();
		// Return an empty list if the motif is not defined.
		if (motif == null || motif == Motif.UNKNOWN) {
			return Collections.emptyList();
		}
		
		List<ArtifactColor> state = getClassifierState();
		ArtifactColor[] motifColors = motifToArtifactColors(motif);
		
		// Return an empty list if the motif has no corresponding color sequence.
		if (motifColors == null || motifColors.length == 0) {
			return Collections.emptyList();
		}
		
		List<ArtifactColor> nextThree = new ArrayList<>();
		int currentSize = state.size();
		
		// Loop three times to get the next three colors.
		for (int i = 0; i < 3; i++) {
			// Use the modulo operator to loop back to the beginning of the motif sequence.
			int index = (currentSize + i) % motifColors.length;
			nextThree.add(motifColors[index]);
		}
		
		return nextThree;
	}
	
	/**
	 * Mirrors a pose across the field centerline for red alliance.
	 * Field width is 144 inches (standard FTC field).
	 * Takes a BLUE pose and returns the mirrored RED pose.
	 * <p>
	 * This is a duplicate of the method in PathRegistry to keep MatchSettings
	 * independent.
	 */
	private Pose mirrorPose(Pose bluePose) {
		return new Pose(
				Settings.Field.WIDTH - bluePose.getX(), // Mirror X coordinate
				bluePose.getY(), // Y stays the same
				Math.PI - bluePose.getHeading() // Mirror heading
		);
	}
	
	public enum AllianceColor {
		RED,
		BLUE,
		UNKNOWN
	}
	
	public enum AutoStartingPosition {
		CLOSE,
		FAR
	}
	
	public enum ArtifactColor {
		GREEN,
		PURPLE,
		UNKNOWN
	}
	
	public enum Motif {
		PPG,
		PGP,
		GPP,
		UNKNOWN
	}
}
