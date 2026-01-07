package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.GREEN_THRESHOLD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.ColorSensor;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

import java.util.ArrayList;
import java.util.List;

/**
 * Test suite for ColorSensor to help tune normalized RGB thresholds.
 * <p>
 * Displays real-time normalized RGB readings and detection results.
 * Useful for:
 * - Tuning GREEN_THRESHOLD and BLUE_THRESHOLD values
 * - Verifying detection accuracy with different artifacts
 * - Collecting sample data for threshold analysis
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li>X: Record current reading (adds to sample history)</li>
 * <li>Y: Clear sample history</li>
 * <li>DPAD LEFT/RIGHT: Switch between left/right sensor (if both
 * configured)</li>
 * </ul>
 * <p>
 * <b>FEATURES:</b>
 * <ul>
 * <li>Real-time normalized RGB values (centered around 1.0)</li>
 * <li>Detection result based on channel thresholds</li>
 * <li>Sample history for data collection</li>
 * <li>Statistics (min/max/avg) for threshold tuning</li>
 * </ul>
 */
@TeleOp(name = "Test: Color Sensor", group = "Tests")
public class ColorSensorTest extends OpMode {
	private static final int MAX_SAMPLES = 50;
	private final List<ColorSample> sampleHistory = new ArrayList<>();
	private ColorSensor[] colorSensors;
	private int currentSensorIndex = 0;
	
	@Override
	public void init() {
		// Build color sensors from hardware map
		List<ColorSensor> sensors = new ArrayList<>();
		
		try {
			ColorSensor left = new ColorSensor(
					Settings.Hardware.COLOR_SENSOR_LEFT.fromHardwareMap(hardwareMap));
			left.init();
			sensors.add(left);
		} catch (Exception e) {
			// Left sensor not configured
		}
		
		try {
			ColorSensor right = new ColorSensor(
					Settings.Hardware.COLOR_SENSOR_RIGHT.fromHardwareMap(hardwareMap));
			right.init();
			sensors.add(right);
		} catch (Exception e) {
			// Right sensor not configured
		}
		
		if (sensors.isEmpty()) {
			telemetry.addLine("❌ ERROR: No color sensors found!");
			telemetry.addLine("Configure COLOR_SENSOR_LEFT or COLOR_SENSOR_RIGHT in Settings");
			telemetry.update();
			return;
		}
		
		colorSensors = sensors.toArray(new ColorSensor[0]);
		
		telemetry.addLine("✅ Color Sensor Test Ready");
		telemetry.addLine();
		telemetry.addLine("Controls:");
		telemetry.addLine("  X: Record sample");
		telemetry.addLine("  Y: Clear history");
		if (colorSensors.length > 1) {
			telemetry.addLine("  DPAD LEFT/RIGHT: Switch sensor");
		}
		telemetry.addLine();
		telemetry.addLine("Current Thresholds:");
		telemetry.addData("  Green Threshold", "%.2f", GREEN_THRESHOLD);
		telemetry.addData("  Blue Threshold", "%.2f", BLUE_THRESHOLD);
		telemetry.update();
	}
	
	@Override
	public void loop() {
		if (colorSensors == null || colorSensors.length == 0) {
			return;
		}
		
		ColorSensor currentSensor = colorSensors[currentSensorIndex];
		
		// Handle controls
		handleControls();
		
		// Get current reading (this also updates the normalized RGB values)
		Artifact detected = currentSensor.getArtifactColor();
		double[] rgb = currentSensor.getNormalizedRgb();
		
		// Record sample if X was pressed
		if (gamepad1.xWasPressed()) {
			recordSample(rgb, detected);
		}
		
		// Display telemetry
		displayTelemetry(rgb, detected);
	}
	
	private void handleControls() {
		// Switch sensor (if multiple available)
		if (colorSensors.length > 1) {
			if (gamepad1.dpadLeftWasPressed()) {
				currentSensorIndex = (currentSensorIndex - 1 + colorSensors.length) % colorSensors.length;
			}
			if (gamepad1.dpadRightWasPressed()) {
				currentSensorIndex = (currentSensorIndex + 1) % colorSensors.length;
			}
		}
		
		// Clear history
		if (gamepad1.yWasPressed()) {
			sampleHistory.clear();
		}
	}
	
	private void recordSample(double[] rgb, Artifact detected) {
		ColorSample sample = new ColorSample(rgb.clone(), detected);
		sampleHistory.add(sample);
		if (sampleHistory.size() > MAX_SAMPLES) {
			sampleHistory.remove(0);
		}
	}
	
	private void displayTelemetry(double[] rgb, Artifact detected) {
		telemetry.addLine("=== CURRENT READING ===");
		if (colorSensors.length > 1) {
			telemetry.addData("Active Sensor", "Sensor #%d", currentSensorIndex + 1);
		}
		telemetry.addLine();
		
		telemetry.addLine("Normalized RGB Values:");
		telemetry.addData("  Red", "%.3f", rgb[0]);
		telemetry.addData("  Green", "%.3f (threshold: %.2f)", rgb[1], GREEN_THRESHOLD);
		telemetry.addData("  Blue", "%.3f (threshold: %.2f)", rgb[2], BLUE_THRESHOLD);
		telemetry.addLine();
		
		// Detection result
		telemetry.addLine("Detection Result:");
		if (detected == Artifact.GREEN) {
			telemetry.addData("  Detected", "✓ GREEN");
			telemetry.addData("  Reason", "Green %.3f >= %.2f", rgb[1], GREEN_THRESHOLD);
		} else if (detected == Artifact.PURPLE) {
			telemetry.addData("  Detected", "✓ PURPLE");
			telemetry.addData("  Reason", "Blue %.3f >= %.2f", rgb[2], BLUE_THRESHOLD);
		} else {
			telemetry.addData("  Detected", "✗ NONE");
			telemetry.addData("  Reason", "Green %.3f < %.2f, Blue %.3f < %.2f",
					rgb[1], GREEN_THRESHOLD, rgb[2], BLUE_THRESHOLD);
		}
		telemetry.addLine();
		
		telemetry.addLine("=== THRESHOLDS ===");
		telemetry.addData("Green Threshold", "%.2f", GREEN_THRESHOLD);
		telemetry.addData("Blue Threshold", "%.2f", BLUE_THRESHOLD);
		telemetry.addLine();
		
		// Sample history and statistics
		if (!sampleHistory.isEmpty()) {
			telemetry.addLine("=== SAMPLE HISTORY ===");
			telemetry.addData("Samples Recorded", "%d / %d", sampleHistory.size(), MAX_SAMPLES);
			
			// Calculate statistics
			calculateAndDisplayStatistics();
			telemetry.addLine();
			
			// Show recent samples
			telemetry.addLine("Recent Samples (X to record):");
			int startIdx = Math.max(0, sampleHistory.size() - 5);
			for (int i = sampleHistory.size() - 1; i >= startIdx; i--) {
				ColorSample sample = sampleHistory.get(i);
				telemetry.addData(
						String.format("#%d", i + 1),
						String.format("%s | R:%.2f G:%.2f B:%.2f",
								sample.detected.color,
								sample.rgb[0], sample.rgb[1], sample.rgb[2]));
			}
			if (sampleHistory.size() > 5) {
				telemetry.addLine("... (" + (sampleHistory.size() - 5) + " more)");
			}
		} else {
			telemetry.addLine("=== SAMPLE HISTORY ===");
			telemetry.addLine("Press X to record samples");
		}
		
		telemetry.update();
	}
	
	private void calculateAndDisplayStatistics() {
		if (sampleHistory.isEmpty()) {
			return;
		}
		
		// Separate samples by detected color
		List<ColorSample> greenSamples = new ArrayList<>();
		List<ColorSample> purpleSamples = new ArrayList<>();
		List<ColorSample> noneSamples = new ArrayList<>();
		
		for (ColorSample sample : sampleHistory) {
			if (sample.detected.color == Artifact.Color.GREEN) {
				greenSamples.add(sample);
			} else if (sample.detected.color == Artifact.Color.PURPLE) {
				purpleSamples.add(sample);
			} else {
				noneSamples.add(sample);
			}
		}
		
		telemetry.addLine();
		telemetry.addLine("Statistics by Color:");
		
		if (!greenSamples.isEmpty()) {
			double avgGreen = greenSamples.stream().mapToDouble(s -> s.rgb[1]).average().orElse(0);
			double minGreen = greenSamples.stream().mapToDouble(s -> s.rgb[1]).min().orElse(0);
			double maxGreen = greenSamples.stream().mapToDouble(s -> s.rgb[1]).max().orElse(0);
			telemetry.addData("  GREEN", "%d samples | G avg:%.2f [%.2f-%.2f]",
					greenSamples.size(), avgGreen, minGreen, maxGreen);
		}
		
		if (!purpleSamples.isEmpty()) {
			double avgBlue = purpleSamples.stream().mapToDouble(s -> s.rgb[2]).average().orElse(0);
			double minBlue = purpleSamples.stream().mapToDouble(s -> s.rgb[2]).min().orElse(0);
			double maxBlue = purpleSamples.stream().mapToDouble(s -> s.rgb[2]).max().orElse(0);
			telemetry.addData("  PURPLE", "%d samples | B avg:%.2f [%.2f-%.2f]",
					purpleSamples.size(), avgBlue, minBlue, maxBlue);
		}
		
		if (!noneSamples.isEmpty()) {
			telemetry.addData("  NONE", "%d samples", noneSamples.size());
		}
	}
	
	private static class ColorSample {
		final double[] rgb;
		final Artifact detected;
		
		ColorSample(double[] rgb, Artifact detected) {
			this.rgb = rgb;
			this.detected = detected;
		}
	}
}
