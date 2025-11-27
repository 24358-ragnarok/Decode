package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.COLOR_SENSOR_DEBOUNCE_TIME;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.software.game.Artifact;

public class ColorUnifier {
	public ColorRangefinder[] sensors;
	public Timer debounce;
	
	public ColorUnifier(ColorRangefinder[] rangefinderArray) {
		this.sensors = rangefinderArray;
		debounce.resetTimer();
	}
	
	public Artifact find() {
		if (debounce.getElapsedTime() < COLOR_SENSOR_DEBOUNCE_TIME) {
			return Artifact.NONE;
		}
		for (ColorRangefinder c : sensors) {
			if (c.isGreenDetected()) {
				debounce.resetTimer();
				return Artifact.GREEN;
			}
			
			if (c.isPurpleDetected()) {
				debounce.resetTimer();
				return Artifact.PURPLE;
			}
		}
		return Artifact.NONE;
	}
	
}
