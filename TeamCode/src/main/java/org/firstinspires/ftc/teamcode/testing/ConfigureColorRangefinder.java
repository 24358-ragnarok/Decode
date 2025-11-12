package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configuration.Settings;

/**
 * Interactive configuration OpMode for the Brushland Labs Color Rangefinder.
 * <p>
 * This OpMode allows real-time tuning of HSV thresholds with live sensor
 * feedback, then writes the configuration to the sensor's persistent memory.
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li>DPAD UP/DOWN: Adjust Purple HSV range (±5)</li>
 * <li>DPAD LEFT/RIGHT: Adjust Green HSV range (±5)</li>
 * <li>LEFT BUMPER: Decrease distance threshold (-5mm)</li>
 * <li>RIGHT BUMPER: Increase distance threshold (+5mm)</li>
 * <li>CROSS (X): Deploy configuration to sensor</li>
 * <li>TRIANGLE (Y): Reset to default values</li>
 * </ul>
 * <p>
 * <b>WORKFLOW:</b>
 * <ol>
 * <li>Connect sensor to I2C port as "Rev Color Sensor V3"</li>
 * <li>Run this OpMode</li>
 * <li>Place colored artifacts in front of sensor</li>
 * <li>Watch real-time HSV values on telemetry</li>
 * <li>Adjust thresholds using gamepad until detection is reliable</li>
 * <li>Press CROSS (X) to write configuration to sensor</li>
 * <li>Wait for LED to blink twice (success indicator)</li>
 * <li>Disconnect from I2C, connect to digital ports</li>
 * </ol>
 * <p>
 * Configuration persists across power cycles.
 */
@TeleOp(name = "Config: Color Rangefinder", group = "Configuration")
public class ConfigureColorRangefinder extends OpMode {
	private RevColorSensorV3 sensor;
	private ColorRangefinderConfigurator configurator;
	
	// Adjustable thresholds (HSV 0-255 scale)
	private double purpleLow;
	private double purpleHigh;
	private double greenLow;
	private double greenHigh;
	private double maxDistance;
	
	// Button press detection
	private boolean lastCross = false;
	private boolean lastTriangle = false;
	private boolean lastDpadUp = false;
	private boolean lastDpadDown = false;
	private boolean lastDpadLeft = false;
	private boolean lastDpadRight = false;
	private boolean lastLeftBumper = false;
	private boolean lastRightBumper = false;
	
	// Deployment status
	private boolean configDeployed = false;
	private double deployTime = 0;
	
	@Override
	public void init() {
		// Initialize sensor on I2C
		sensor = hardwareMap.get(RevColorSensorV3.class, Settings.HardwareIDs.TRANSFER_COLOR_SENSOR);
		sensor.initialize();
		configurator = new ColorRangefinderConfigurator(sensor);
		
		// Load default values from Settings
		resetToDefaults();
		
		telemetry.addLine("Interactive Color Rangefinder Config");
		telemetry.addLine();
		telemetry.addLine("Controls:");
		telemetry.addLine("  DPAD UP/DOWN = Purple range");
		telemetry.addLine("  DPAD LEFT/RIGHT = Green range");
		telemetry.addLine("  BUMPERS = Distance threshold");
		telemetry.addLine("  X = Deploy config to sensor");
		telemetry.addLine("  Y = Reset to defaults");
		telemetry.addLine();
		telemetry.addLine("Ready! Press START to begin");
		telemetry.update();
	}
	
	@Override
	public void loop() {
		// Read current sensor values
		NormalizedRGBA colors = sensor.getNormalizedColors();
		double distance = sensor.getDistance(DistanceUnit.MM);
		
		// Calculate HSV (simplified - just hue for now)
		double hue = rgbToHue(colors.red, colors.green, colors.blue);
		
		// Handle gamepad input for threshold adjustment
		handleThresholdAdjustment();
		
		// Handle deployment
		if (gamepad1.cross && !lastCross) {
			deployConfiguration();
		}
		lastCross = gamepad1.cross;
		
		// Handle reset
		if (gamepad1.triangle && !lastTriangle) {
			resetToDefaults();
		}
		lastTriangle = gamepad1.triangle;
		
		// Display telemetry
		displayTelemetry(colors, distance, hue);
	}
	
	private void handleThresholdAdjustment() {
		// Purple range adjustment (DPAD UP/DOWN)
		if (gamepad1.dpad_up && !lastDpadUp) {
			purpleHigh = Math.min(255, purpleHigh + 5);
		}
		if (gamepad1.dpad_down && !lastDpadDown) {
			purpleLow = Math.max(0, purpleLow - 5);
		}
		lastDpadUp = gamepad1.dpad_up;
		lastDpadDown = gamepad1.dpad_down;
		
		// Green range adjustment (DPAD LEFT/RIGHT)
		if (gamepad1.dpad_right && !lastDpadRight) {
			greenHigh = Math.min(255, greenHigh + 5);
		}
		if (gamepad1.dpad_left && !lastDpadLeft) {
			greenLow = Math.max(0, greenLow - 5);
		}
		lastDpadLeft = gamepad1.dpad_left;
		lastDpadRight = gamepad1.dpad_right;
		
		// Distance threshold adjustment (BUMPERS)
		if (gamepad1.right_bumper && !lastRightBumper) {
			maxDistance = Math.min(100, maxDistance + 5);
		}
		if (gamepad1.left_bumper && !lastLeftBumper) {
			maxDistance = Math.max(5, maxDistance - 5);
		}
		lastLeftBumper = gamepad1.left_bumper;
		lastRightBumper = gamepad1.right_bumper;
	}
	
	private void deployConfiguration() {
		try {
			// Configure Pin 0 for PURPLE detection
			configurator.setPin0Digital(
					ColorRangefinderConfigurator.DigitalMode.HSV,
					purpleLow,
					purpleHigh);
			configurator.setPin0DigitalMaxDistance(maxDistance);
			
			// Configure Pin 1 for GREEN detection
			configurator.setPin1Digital(
					ColorRangefinderConfigurator.DigitalMode.HSV,
					greenLow,
					greenHigh);
			configurator.setPin1DigitalMaxDistance(maxDistance);
			
			configDeployed = true;
			deployTime = getRuntime();
			
			// Haptic feedback
			gamepad1.rumble(500);
			
		} catch (Exception e) {
			telemetry.addLine("❌ DEPLOYMENT FAILED: " + e.getMessage());
		}
	}
	
	private void resetToDefaults() {
		purpleLow = Settings.ColorSensor.PURPLE_HUE_LOW;
		purpleHigh = Settings.ColorSensor.PURPLE_HUE_HIGH;
		greenLow = Settings.ColorSensor.GREEN_HUE_LOW;
		greenHigh = Settings.ColorSensor.GREEN_HUE_HIGH;
		maxDistance = Settings.ColorSensor.MAX_DETECTION_DISTANCE_MM;
		configDeployed = false;
	}
	
	private void displayTelemetry(NormalizedRGBA colors, double distance, double hue) {
		telemetry.clear();
		
		// Deployment status
		if (configDeployed && (getRuntime() - deployTime) < 3.0) {
			telemetry.addLine("✓ CONFIG DEPLOYED!");
			telemetry.addLine("Watch for sensor LED to blink twice");
			telemetry.addLine();
		}
		
		// Live sensor readings
		telemetry.addLine("=== LIVE SENSOR DATA ===");
		telemetry.addData("Distance", "%.1f mm", distance);
		telemetry.addData("RGB", "R:%.2f G:%.2f B:%.2f", colors.red, colors.green, colors.blue);
		telemetry.addData("HSV Hue", "%.0f (%.0f°)", hue, hue / 255.0 * 360.0);
		telemetry.addLine();
		
		// Detection simulation
		boolean purpleMatch = hue >= purpleLow && hue <= purpleHigh && distance <= maxDistance;
		boolean greenMatch = hue >= greenLow && hue <= greenHigh && distance <= maxDistance;
		
		telemetry.addLine("=== DETECTION PREVIEW ===");
		telemetry.addData("Purple", purpleMatch ? "✓ WOULD DETECT" : "✗ No match");
		telemetry.addData("Green", greenMatch ? "✓ WOULD DETECT" : "✗ No match");
		telemetry.addLine();
		
		// Current thresholds
		telemetry.addLine("=== CURRENT THRESHOLDS ===");
		telemetry.addData("Purple Range", "%.0f-%.0f (%.0f°-%.0f°)",
				purpleLow, purpleHigh,
				purpleLow / 255.0 * 360.0, purpleHigh / 255.0 * 360.0);
		telemetry.addData("Green Range", "%.0f-%.0f (%.0f°-%.0f°)",
				greenLow, greenHigh,
				greenLow / 255.0 * 360.0, greenHigh / 255.0 * 360.0);
		telemetry.addData("Max Distance", "%.0f mm", maxDistance);
		telemetry.addLine();
		
		// Controls reminder
		telemetry.addLine("=== CONTROLS ===");
		telemetry.addLine("DPAD ↑↓ = Purple | DPAD ←→ = Green");
		telemetry.addLine("BUMPERS = Distance | X = Deploy | Y = Reset");
		
		telemetry.update();
	}
	
	/**
	 * Converts RGB to HSV hue value (0-255 scale).
	 * Simplified calculation for hue only.
	 */
	private double rgbToHue(double r, double g, double b) {
		double max = Math.max(r, Math.max(g, b));
		double min = Math.min(r, Math.min(g, b));
		double delta = max - min;
		
		if (delta == 0) {
			return 0;
		}
		
		double hue;
		if (max == r) {
			hue = 60 * (((g - b) / delta) % 6);
		} else if (max == g) {
			hue = 60 * (((b - r) / delta) + 2);
		} else {
			hue = 60 * (((r - g) / delta) + 4);
		}
		
		if (hue < 0) {
			hue += 360;
		}
		
		// Convert from 0-360 to 0-255 scale
		return hue / 360.0 * 255.0;
	}
}

/**
 * Helper class for writing configuration to the Brushland Labs Color
 * Rangefinder.
 * Based on official documentation from Brushland Labs.
 */
class ColorRangefinderConfigurator {
	private final I2cDeviceSynchSimple i2c;
	
	public ColorRangefinderConfigurator(RevColorSensorV3 emulator) {
		this.i2c = emulator.getDeviceClient();
		this.i2c.enableWriteCoalescing(true);
	}
	
	/**
	 * Configure Pin 0 to be in digital mode and add a threshold.
	 * For HSV colors, bounds should be from 0-255 (scaled from 0-360°).
	 * For distance, bounds should be from 0-100 (mm).
	 */
	public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
		setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
	}
	
	/**
	 * Configure Pin 1 to be in digital mode and add a threshold.
	 * For HSV colors, bounds should be from 0-255 (scaled from 0-360°).
	 * For distance, bounds should be from 0-100 (mm).
	 */
	public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
		setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
	}
	
	/**
	 * Sets the maximum distance (in millimeters) within which an object must be
	 * located for Pin 0's thresholds to trigger.
	 */
	public void setPin0DigitalMaxDistance(double mmRequirement) {
		setPin0Digital(DigitalMode.DISTANCE, mmRequirement, mmRequirement);
	}
	
	/**
	 * Sets the maximum distance (in millimeters) within which an object must be
	 * located for Pin 1's thresholds to trigger.
	 */
	public void setPin1DigitalMaxDistance(double mmRequirement) {
		setPin1Digital(DigitalMode.DISTANCE, mmRequirement, mmRequirement);
	}
	
	/**
	 * Low-level method to write digital configuration to the sensor.
	 */
	private void setDigital(PinNum pin, DigitalMode mode, double lowerBound, double higherBound) {
		byte lower = (byte) (lowerBound);
		byte higher = (byte) (higherBound);
		i2c.write(pin.modeAddress, new byte[]{mode.value, lower, higher});
	}
	
	private enum PinNum {
		PIN0(0x28),
		PIN1(0x2D);
		
		private final byte modeAddress;
		
		PinNum(int modeAddress) {
			this.modeAddress = (byte) modeAddress;
		}
	}
	
	public enum DigitalMode {
		RED(1),
		BLUE(2),
		GREEN(3),
		ALPHA(4),
		HSV(5),
		DISTANCE(6);
		
		public final byte value;
		
		DigitalMode(int value) {
			this.value = (byte) value;
		}
	}
}
