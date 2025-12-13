package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.COLOR_SENSOR_DEBOUNCE_TIME;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

public class ColorUnifier {
	public ColorRangefinder[] sensors;
	public MechanismManager mechanisms;
	public Timer debounce;
	
	public ColorUnifier(MechanismManager mechanisms, ColorRangefinder[] rangefinderArray) {
		this.mechanisms = mechanisms; // FIX: Assign the passed manager
		this.sensors = rangefinderArray;
		this.debounce = new Timer(); // FIX: Instantiate the object
		// debounce.resetTimer(); // Not strictly necessary on new(), but harmless
	}
	
	public Artifact find() {
		if (debounce.getElapsedTime() < COLOR_SENSOR_DEBOUNCE_TIME) {
			return Artifact.NONE;
		}
		
		for (ColorRangefinder c : sensors) {
			if (c.isGreenDetected()) {
				debounce.resetTimer();
				
				VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
				
				if (transfer != null) {
					return new Artifact(Artifact.Color.GREEN, transfer.getTicks());
				} else {
					// Fallback if transfer mechanism is missing/invalid
					return new Artifact(Artifact.Color.GREEN, 0);
				}
			}
			
			if (c.isPurpleDetected()) {
				debounce.resetTimer();
				
				VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
				
				if (transfer != null) {
					return new Artifact(Artifact.Color.PURPLE, transfer.getTicks());
				} else {
					// Fallback if transfer mechanism is missing/invalid
					return new Artifact(Artifact.Color.PURPLE, 0);
				}
			}
		}
		return Artifact.NONE;
	}
}