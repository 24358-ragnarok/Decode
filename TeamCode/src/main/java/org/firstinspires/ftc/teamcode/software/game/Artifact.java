package org.firstinspires.ftc.teamcode.software.game;

import androidx.annotation.NonNull;

import java.util.Objects;

public class Artifact {
	// Static instances for clean pattern definition in Motif
	public static final Artifact GREEN = new Artifact(Color.GREEN, 0);
	public static final Artifact PURPLE = new Artifact(Color.PURPLE, 0);
	public static final Artifact NONE = new Artifact(Color.NONE, 0);
	
	public int transferTicksWhenAtEntrance;
	public Color color;
	
	// Default constructor for an empty artifact
	public Artifact() {
		this.color = Color.NONE;
		this.transferTicksWhenAtEntrance = 0;
	}
	
	// Constructor to set color and ticks (used when reading from configuration or placing)
	public Artifact(Color color, int ticks) {
		this.color = color;
		this.transferTicksWhenAtEntrance = ticks;
	}
	
	@NonNull
	@Override
	public String toString() {
		if (color == Color.NONE) return "NONE";
		return String.format(
				"%s ball which started at %.2f ticks",
				color.name().charAt(0) + color.name().substring(1).toLowerCase(),
				(double) transferTicksWhenAtEntrance
		);
	}
	
	/**
	 * Determines equality based ONLY on the color, which is essential for Motif pattern comparison.
	 */
	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		Artifact artifact = (Artifact) o;
		return color == artifact.color;
	}
	
	@Override
	public int hashCode() {
		return Objects.hash(color);
	}
	
	public enum Color {
		GREEN,
		PURPLE,
		NONE
	}
}