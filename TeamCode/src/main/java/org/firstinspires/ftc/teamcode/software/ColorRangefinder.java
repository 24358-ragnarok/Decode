package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

/**
 * Color Rangefinder wrapper for reading digital outputs from a pre-configured
 * Brushland Labs Color Rangefinder sensor.
 * <p>
 * <b>PREREQUISITE:</b> The sensor MUST be configured using the
 * {@code ConfigureColorRangefinder} OpMode BEFORE using this class. After
 * configuration, physically disconnect the sensor from the I2C port and connect
 * it to digital input ports on the hub.
 * <p>
 * The sensor's onboard processor handles all HSV color detection and distance
 * filtering in hardware. This class simply reads the digital pin states:
 * - Pin 0 → HIGH when PURPLE detected (HSV hue ~160-190°, within distance)
 * - Pin 1 → HIGH when GREEN detected (HSV hue ~110-140°, within distance)
 * <p>
 * Benefits:
 * - Ultra-fast detection (&lt;1ms, no I2C communication)
 * - HSV is lighting-independent (more reliable than RGB)
 * - Hardware distance filtering eliminates false positives
 * - Zero CPU overhead for color processing
 * <p>
 * Configuration is persistent across power cycles. Re-configuration is only
 * needed when changing detection thresholds.
 * <p>
 * Documentation: https://docs.brushlandlabs.com/sensors/color-rangefinder/
 */
public class ColorRangefinder {
	private final DigitalChannel pin0;
	private final DigitalChannel pin1;
	
	/**
	 * Constructs a ColorRangefinder for reading digital outputs.
	 * <p>
	 * <b>SETUP REQUIRED:</b>
	 * <ol>
	 * <li>Run {@code ConfigureColorRangefinder} OpMode with sensor on I2C port</li>
	 * <li>Wait for sensor LED to blink twice (indicates successful
	 * configuration)</li>
	 * <li>Physically disconnect sensor from I2C port</li>
	 * <li>Connect sensor to digital ports configured in hardware map</li>
	 * <li>Sensor configuration persists - only needed once per settings change</li>
	 * </ol>
	 * <p>
	 * Hardware map must include:
	 * - "colorPin0" → Digital input for purple detection
	 * - "colorPin1" → Digital input for green detection
	 *
	 * @param hardwareMap The hardware map from the OpMode
	 */
	public ColorRangefinder(HardwareMap hardwareMap, String pin0Name, String pin1Name) {
		this.pin0 = hardwareMap.get(DigitalChannel.class, pin0Name);
		this.pin1 = hardwareMap.get(DigitalChannel.class, pin1Name);
		
		pin0.setMode(DigitalChannel.Mode.INPUT);
		pin1.setMode(DigitalChannel.Mode.INPUT);
	}
	
	/**
	 * Gets the currently detected artifact color based on digital pin states.
	 * <p>
	 * Reads the hardware digital outputs updated by the sensor's onboard
	 * processor. No I2C communication - instant response time.
	 * <p>
	 * The sensor outputs LOW (false from getState()) when a color is detected.
	 *
	 * @return The detected artifact color (GREEN, PURPLE, or UNKNOWN)
	 */
	public MatchSettings.ArtifactColor getArtifactColor() {
		boolean purpleDetected = isPurpleDetected();
		boolean greenDetected = isGreenDetected();
		
		if (purpleDetected && !greenDetected) {
			return MatchSettings.ArtifactColor.PURPLE;
		} else if (greenDetected && !purpleDetected) {
			return MatchSettings.ArtifactColor.GREEN;
		} else {
			// Both or neither detected
			return MatchSettings.ArtifactColor.UNKNOWN;
		}
	}
	
	/**
	 * Checks if PURPLE artifact is detected.
	 * <p>
	 * Reads Pin 0 digital state. The sensor pulls the pin HIGH when purple is
	 * detected within the configured HSV hue range and distance threshold.
	 *
	 * @return true if purple is detected within configured thresholds
	 */
	public boolean isPurpleDetected() {
		// Digital pins are active HIGH: getState() returns true when sensor detects
		// color
		return pin0.getState();
	}
	
	/**
	 * Checks if GREEN artifact is detected.
	 * <p>
	 * Reads Pin 1 digital state. The sensor pulls the pin HIGH when green is
	 * detected within the configured HSV hue range and distance threshold.
	 *
	 * @return true if green is detected within configured thresholds
	 */
	public boolean isGreenDetected() {
		// Digital pins are active HIGH: getState() returns true when sensor detects
		// color
		return pin1.getState();
	}
}