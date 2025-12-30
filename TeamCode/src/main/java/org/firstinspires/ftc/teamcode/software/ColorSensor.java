package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.CONFIDENCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.GREEN_TARGET;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.PURPLE_TARGET;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.software.game.Artifact;

/**
 * A Decode-specific Color Sensor wrapper to accurately determine what artifact
 * type is in front of the sensor.
 * <p>
 * Uses RGB color distance calculations to classify artifacts as GREEN, PURPLE,
 * or UNKNOWN.
 * Color thresholds and confidence values can be tuned in Settings.ColorSensor.
 * <p>
 * The sensor uses Euclidean distance in RGB space to determine the closest
 * match
 * to predefined color targets, with a confidence threshold to reject uncertain
 * readings.
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
		((LynxI2cDeviceSynch) colorSensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
		colorSensor.initialize();
		colorSensor.enableLed(true);
	}
	
	/**
	 * Updates the data and checks if there is a desired object detected
	 *
	 * @return detected artifact color
	 */
	public Artifact getArtifactColor() {
		rgbValues[0] = colorSensor.red();
		rgbValues[1] = colorSensor.green();
		rgbValues[2] = colorSensor.blue();
		
		double greenConfidence = computeDistance(rgbValues, GREEN_TARGET);
		double purpleConfidence = computeDistance(rgbValues, PURPLE_TARGET);
		
		if (greenConfidence < purpleConfidence && greenConfidence < CONFIDENCE_THRESHOLD) {
			return Artifact.GREEN;
		} else if (purpleConfidence < greenConfidence && purpleConfidence < CONFIDENCE_THRESHOLD) {
			return Artifact.PURPLE;
		} else {
			return Artifact.NONE;
		}
	}
	
	/**
	 * Calculates the Euclidean distance between measured RGB values and target RGB
	 * values.
	 * Lower distances indicate better color matches.
	 *
	 * @param measured The measured RGB values from the sensor
	 * @param target   The target RGB values to compare against
	 * @return The Euclidean distance between the two color points
	 */
	public double computeDistance(double[] measured, double[] target) {
		double dr = measured[0] - target[0];
		double dg = measured[1] - target[1];
		double db = measured[2] - target[2];
		return Math.sqrt(dr * dr + dg * dg + db * db);
	}
}