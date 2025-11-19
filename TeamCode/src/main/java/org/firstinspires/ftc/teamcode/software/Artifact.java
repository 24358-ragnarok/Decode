package org.firstinspires.ftc.teamcode.software;

import androidx.annotation.NonNull;

public class Artifact {
	public int transferTicksWhenAtEntrance;
	public Color color;
	
	public Artifact() {
		color = Color.NONE;
		transferTicksWhenAtEntrance = 0;
	}
	
	public Artifact(Color color, int ticks) {
		this.color = color;
		transferTicksWhenAtEntrance = ticks;
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
	
	
	public enum Color {
		GREEN,
		PURPLE,
		NONE
	}
}
