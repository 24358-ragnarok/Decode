package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.GREEN_THRESHOLD;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.software.game.Artifact;

/**
 * A Decode-specific Color Sensor wrapper to accurately determine what artifact
 * type is in front of the sensor.
 * <p>
 * Uses normalized RGB values (each channel divided by average) to classify
 * artifacts as GREEN, PURPLE, or NONE.
 * <p>
 * Normalization gives each channel a value around 1.0 for neutral colors,
 * with dominant channels exceeding 1.0 proportionally.
 */
public class ColorSensor {
	private static final int RGB_COMPONENTS = 3;
	private final double[] normalizedRgb = new double[RGB_COMPONENTS];
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
	 * Updates sensor readings and returns the detected artifact color.
	 * <p>
	 * Detection logic:
	 * - If normalized green >= GREEN_THRESHOLD → GREEN
	 * - Else if normalized blue >= BLUE_THRESHOLD → PURPLE
	 * - Else → NONE
	 *
	 * @return detected artifact color
	 */
	public Artifact getArtifactColor() {
		updateNormalizedRgb();
		
		if (normalizedRgb[1] >= GREEN_THRESHOLD) {
			return Artifact.GREEN;
		} else if (normalizedRgb[2] >= BLUE_THRESHOLD) {
			return Artifact.PURPLE;
		} else {
			return Artifact.NONE;
		}
	}
	
	/**
	 * Updates the normalized RGB values from the sensor.
	 * Each channel is divided by the average of all channels (sum / 3),
	 * resulting in values centered around 1.0.
	 */
	private void updateNormalizedRgb() {
		double r = colorSensor.red();
		double g = colorSensor.green();
		double b = colorSensor.blue();
		double sum = r + g + b;
		
		if (sum > 0) {
			normalizedRgb[0] = r / sum * 3;
			normalizedRgb[1] = g / sum * 3;
			normalizedRgb[2] = b / sum * 3;
		} else {
			normalizedRgb[0] = 0;
			normalizedRgb[1] = 0;
			normalizedRgb[2] = 0;
		}
	}
	
	/**
	 * Returns the normalized RGB values from the last sensor reading.
	 * Values are centered around 1.0, with dominant channels exceeding 1.0.
	 * <p>
	 * Index 0 = Red, Index 1 = Green, Index 2 = Blue
	 *
	 * @return array of normalized RGB values [R, G, B]
	 */
	public double[] getNormalizedRgb() {
		return normalizedRgb;
	}
}