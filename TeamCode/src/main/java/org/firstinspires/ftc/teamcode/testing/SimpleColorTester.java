package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.ColorRangefinder;

/**
 * Test OpMode for the Brushland Labs Color Rangefinder sensor.
 * <p>
 * Displays hardware-accelerated HSV detection results from digital pins.
 * <p>
 * <b>PREREQUISITES:</b>
 * <ol>
 * <li>Run "Config: Color Rangefinder" OpMode with sensor on I2C port</li>
 * <li>Wait for sensor LED to blink twice</li>
 * <li>Disconnect sensor from I2C port</li>
 * <li>Connect sensor to digital ports (colorPin0, colorPin1)</li>
 * <li>Run this test OpMode</li>
 * </ol>
 * <p>
 * The sensor processes colors using onboard HSV detection and outputs digital
 * signals. This is ultra-fast (no I2C) and lighting-independent.
 */
@TeleOp(name = "Test: Digital Pin Color Rangefinder", group = "Tests")
public class SimpleColorTester extends OpMode {
	private ColorRangefinder colorRangefinder;
	
	@Override
	public void init() {
		// Initialize ColorRangefinder (reads digital pins only)
		colorRangefinder = new ColorRangefinder(hardwareMap, "test0", "test1");
		
		telemetry.addLine("Color Rangefinder Test Ready");
		telemetry.addLine();
		telemetry.addLine("⚠ Sensor must be pre-configured!");
		telemetry.addLine("Run 'Config: Color Rangefinder' first");
		telemetry.addLine();
		telemetry.addLine("Configured Detection:");
		telemetry.addData("Purple", "HSV %.0f°-%.0f° (%.0f-%.0f/255)",
				160.0, 190.0, Settings.ColorSensor.PURPLE_HUE_LOW, Settings.ColorSensor.PURPLE_HUE_HIGH);
		telemetry.addData("Green", "HSV %.0f°-%.0f° (%.0f-%.0f/255)",
				110.0, 140.0, Settings.ColorSensor.GREEN_HUE_LOW, Settings.ColorSensor.GREEN_HUE_HIGH);
		telemetry.addData("Max Distance", "%.0f mm", Settings.ColorSensor.MAX_DETECTION_DISTANCE_MM);
		telemetry.update();
	}
	
	@Override
	public void loop() {
		// Get hardware-accelerated detection results (digital pin reads)
		boolean purpleDetected = colorRangefinder.isPurpleDetected();
		boolean greenDetected = colorRangefinder.isGreenDetected();
		
		// Display detection results
		telemetry.addLine("=== DETECTION RESULTS ===");
		telemetry.addData("Artifact Color", colorRangefinder.getArtifactColor());
		telemetry.addLine();
		telemetry.addData("Purple (Pin 0)", purpleDetected ? "✓ DETECTED" : "✗ Not detected");
		telemetry.addData("Green (Pin 1)", greenDetected ? "✓ DETECTED" : "✗ Not detected");
		telemetry.addLine();
		
		// Display configuration reminder
		telemetry.addLine("=== CONFIGURATION ===");
		telemetry.addData("Purple HSV", "%.0f-%.0f/255",
				Settings.ColorSensor.PURPLE_HUE_LOW, Settings.ColorSensor.PURPLE_HUE_HIGH);
		telemetry.addData("Green HSV", "%.0f-%.0f/255",
				Settings.ColorSensor.GREEN_HUE_LOW, Settings.ColorSensor.GREEN_HUE_HIGH);
		telemetry.addLine();
		
		telemetry.update();
	}
}
