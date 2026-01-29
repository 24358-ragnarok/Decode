package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.COLOR_SENSOR_DEBOUNCE_TIME_MS;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

public class ColorUnifier {
	private final ColorRangefinder[] rangefinders;
	private final ColorSensor[] colorSensors;
	private final MechanismManager mechanisms;
	private final Timer debounce;
	
	public ColorUnifier(MechanismManager mechanisms,
	                    ColorRangefinder[] rangefinderArray,
	                    ColorSensor[] colorSensorArray) {
		this.mechanisms = mechanisms;
		this.rangefinders = rangefinderArray != null ? rangefinderArray : new ColorRangefinder[0];
		this.colorSensors = colorSensorArray != null ? colorSensorArray : new ColorSensor[0];
		this.debounce = new Timer();
	}
	
	/**
	 * Returns a detected artifact from any available color sensors.
	 * Priority: rangefinders (fast digital) first, then RGB color sensors.
	 */
	public Artifact find() {
		if (debounce.getElapsedTime() < COLOR_SENSOR_DEBOUNCE_TIME_MS) {
			return Artifact.NONE;
		}
		
		Artifact detected = detectWithRangefinders();
		if (detected == Artifact.NONE) {
			detected = detectWithColorSensors();
		}
		
		if (detected != Artifact.NONE) {
			debounce.resetTimer();
			return withTransferTicks(detected.color);
		}
		
		return Artifact.NONE;
	}
	
	private Artifact detectWithRangefinders() {
		for (ColorRangefinder c : rangefinders) {
			if (c.isGreenDetected()) {
				return Artifact.GREEN;
			}
			if (c.isPurpleDetected()) {
				return Artifact.PURPLE;
			}
		}
		return Artifact.NONE;
	}
	
	private Artifact detectWithColorSensors() {
		for (ColorSensor sensor : colorSensors) {
			Artifact result = sensor.getArtifactColor();
			if (result == Artifact.GREEN) {
				return Artifact.GREEN;
			} else if (result == Artifact.PURPLE) {
				return Artifact.PURPLE;
			}
		}
		return Artifact.NONE;
	}
	
	private Artifact withTransferTicks(Artifact.Color color) {
		VerticalWheelTransfer transfer = mechanisms.get(VerticalWheelTransfer.class);
		if (transfer != null) {
			return new Artifact(color, transfer.getTicks());
		}
		return new Artifact(color, 0);
	}
}