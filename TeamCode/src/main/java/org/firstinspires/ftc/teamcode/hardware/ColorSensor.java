package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.CONFIDENCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.GREEN_TARGET;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.PURPLE_TARGET;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

/**
 * A Decode-specific Color Sensor wrapper to accurately determine what artifact
 * type is in front
 * of the sensor.
 * TODO: Tune color thresholds and confidence values in Settings.ColorSensor
 */
public class ColorSensor {
	private static final int RGB_COMPONENTS = 3;
	private final double[] rgbValues = new double[RGB_COMPONENTS];
	private final RevColorSensorV3 colorSensor;
	
	public ColorSensor(RevColorSensorV3 colorSensorV3) {
		this.colorSensor = colorSensorV3;
	}
	
	/**
	 * Initializes Color Sensor and enables LED
	 */
	public final void init() {
		colorSensor.initialize();
		colorSensor.enableLed(true);
	}
	
	/**
	 * Updates the data and checks if there is a desired object detected
	 *
	 * @return detected artifact color
	 */
	public MatchSettings.ArtifactColor getArtifactColor() {
		rgbValues[0] = colorSensor.red();
		rgbValues[1] = colorSensor.green();
		rgbValues[2] = colorSensor.blue();
		
		double greenConfidence = computeDistance(rgbValues, GREEN_TARGET);
		double purpleConfidence = computeDistance(rgbValues, PURPLE_TARGET);
		
		if (greenConfidence < purpleConfidence && greenConfidence < CONFIDENCE_THRESHOLD) {
			return MatchSettings.ArtifactColor.GREEN;
		} else if (purpleConfidence < greenConfidence && purpleConfidence < CONFIDENCE_THRESHOLD) {
			return MatchSettings.ArtifactColor.PURPLE;
		} else {
			return MatchSettings.ArtifactColor.UNKNOWN;
		}
	}
	
	/**
	 * Euclidean distance between measured RGB and target RGB
	 */
	public double computeDistance(double[] measured, double[] target) {
		double dr = measured[0] - target[0];
		double dg = measured[1] - target[1];
		double db = measured[2] - target[2];
		return Math.sqrt(dr * dr + dg * dg + db * db);
	}
}