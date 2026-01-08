package org.firstinspires.ftc.teamcode.software.game;

import java.util.Arrays;

/**
 * Represents the arrangement of three Artifacts in the motif pattern (GPP, PGP, PPG).
 * This class is responsible for converting between the physical artifact pattern
 * and the AprilTag ID used for detection.
 */
public class Motif {
	// Static fields defining the three specific motif patterns using Artifact's static instances
	public static final Motif GPP = new Motif(new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
	public static final Motif PGP = new Motif(new Artifact[]{Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE});
	public static final Motif PPG = new Motif(new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN});
	// An Unknown/Reset motif for cases where detection fails or is irrelevant
	public static final Motif UNKNOWN = new Motif(new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE});
	public final Artifact[] state;
	
	public Motif(Artifact[] artifacts) {
		if (artifacts == null || artifacts.length != 3) {
			throw new IllegalArgumentException("Motif must consist of exactly three Artifacts.");
		}
		this.state = artifacts;
	}
	
	/**
	 * Creates a Motif object from an AprilTag ID.
	 *
	 * @param apriltagValue The ID detected (21, 22, or 23).
	 * @return The corresponding static Motif object (GPP, PGP, or PPG).
	 */
	public static Motif fromApriltag(int apriltagValue) {
		switch (apriltagValue) {
			case 21:
				return GPP;
			case 22:
				return PGP;
			case 23:
				return PPG;
			default:
				// If the ID is not one of the three motif tags, return UNKNOWN.
				return UNKNOWN;
		}
	}
	
	/**
	 * Converts the Motif pattern to its corresponding AprilTag ID.
	 *
	 * @return The AprilTag ID (21, 22, or 23), or 0 if the pattern is unknown or non-standard.
	 */
	public int toApriltag() {
		// Arrays.equals uses the Artifact.equals() method (which only compares color) to check the elements.
		if (Arrays.equals(this.state, GPP.state)) {
			return 21;
		} else if (Arrays.equals(this.state, PGP.state)) {
			return 22;
		} else if (Arrays.equals(this.state, PPG.state)) {
			return 23;
		}
		return 0; // Represents an unknown/non-standard motif
	}
	
	@Override
	public String toString() {
		return "Motif: [" + this.state[0].color + ", " + this.state[1].color + ", " + this.state[2].color + "]";
	}
}