package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.CONFIDENCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.GREEN_TARGET;
import static org.firstinspires.ftc.teamcode.configuration.Settings.ColorSensor.PURPLE_TARGET;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.ColorSensor;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

import java.util.ArrayList;
import java.util.List;

/**
 * Comprehensive test suite for ColorSensor to help tune color bounds and
 * confidence thresholds.
 * <p>
 * Displays real-time RGB readings, distance calculations, and detection
 * results.
 * Useful for:
 * - Tuning GREEN_TARGET and PURPLE_TARGET RGB values
 * - Adjusting CONFIDENCE_THRESHOLD
 * - Verifying detection accuracy with different artifacts
 * - Collecting sample data for analysis
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li>X: Record current reading (adds to sample history)</li>
 * <li>Y: Clear sample history</li>
 * <li>DPAD UP/DOWN: Adjust confidence threshold (+/- 5)</li>
 * <li>DPAD LEFT/RIGHT: Switch between left/right sensor (if both
 * configured)</li>
 * </ul>
 * <p>
 * <b>FEATURES:</b>
 * <ul>
 * <li>Real-time RGB values and raw sensor readings</li>
 * <li>Distance calculations to both color targets</li>
 * <li>Detection result with confidence indicators</li>
 * <li>Sample history for data collection</li>
 * <li>Statistics (min/max/avg distances) for tuning</li>
 * </ul>
 */
@TeleOp(name = "Test: Color Sensor", group = "Tests")
public class ColorSensorTest extends OpMode {
	private static final int MAX_SAMPLES = 50;
	private final List<ColorSample> sampleHistory = new ArrayList<>();
	private final boolean lastXState = false;
	private ColorSensor[] colorSensors;
	private int currentSensorIndex = 0;
	private double adjustedConfidenceThreshold = CONFIDENCE_THRESHOLD;
	
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
		telemetry.addLine("  DPAD UP/DOWN: Adjust threshold");
		if (colorSensors.length > 1) {
			telemetry.addLine("  DPAD LEFT/RIGHT: Switch sensor");
		}
		telemetry.addLine();
		telemetry.addLine("Current Settings:");
		telemetry.addData("Green Target", "RGB(%.0f, %.0f, %.0f)",
				GREEN_TARGET[0], GREEN_TARGET[1], GREEN_TARGET[2]);
		telemetry.addData("Purple Target", "RGB(%.0f, %.0f, %.0f)",
				PURPLE_TARGET[0], PURPLE_TARGET[1], PURPLE_TARGET[2]);
		telemetry.addData("Confidence Threshold", "%.1f", CONFIDENCE_THRESHOLD);
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
		
		// Get current reading
		Artifact detected = currentSensor.getArtifactColor();
		double[] rgb = getCurrentRGB(currentSensor);
		double greenDistance = currentSensor.computeDistance(rgb, GREEN_TARGET);
		double purpleDistance = currentSensor.computeDistance(rgb, PURPLE_TARGET);
		
		// Record sample if X was pressed
		if (gamepad1.xWasPressed()) {
			recordSample(rgb, greenDistance, purpleDistance, detected);
		}
		
		// Display telemetry
		displayTelemetry(currentSensor, rgb, greenDistance, purpleDistance, detected);
	}
	
	private void handleControls() {
		// Adjust confidence threshold
		if (gamepad1.dpadUpWasPressed()) {
			adjustedConfidenceThreshold = Math.min(200, adjustedConfidenceThreshold + 5);
		}
		if (gamepad1.dpadDownWasPressed()) {
			adjustedConfidenceThreshold = Math.max(0, adjustedConfidenceThreshold - 5);
		}
		
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
	
	private double[] getCurrentRGB(ColorSensor sensor) {
		// Access RGB values through reflection or create a method in ColorSensor
		// For now, we'll need to add a getter method or use reflection
		// This is a limitation - we need to modify ColorSensor to expose RGB values
		// For testing purposes, we'll compute from the artifact detection
		// Actually, let's read the sensor directly using reflection or add a getter
		try {
			java.lang.reflect.Field rgbField = ColorSensor.class.getDeclaredField("rgbValues");
			rgbField.setAccessible(true);
			return (double[]) rgbField.get(sensor);
		} catch (Exception e) {
			// Fallback: return zeros if reflection fails
			return new double[]{0, 0, 0};
		}
	}
	
	private void recordSample(double[] rgb, double greenDist, double purpleDist, Artifact detected) {
		ColorSample sample = new ColorSample(rgb.clone(), greenDist, purpleDist, detected);
		sampleHistory.add(sample);
		if (sampleHistory.size() > MAX_SAMPLES) {
			sampleHistory.remove(0);
		}
	}
	
	private void displayTelemetry(ColorSensor sensor, double[] rgb, double greenDist,
	                              double purpleDist, Artifact detected) {
		telemetry.addLine("=== CURRENT READING ===");
		if (colorSensors.length > 1) {
			telemetry.addData("Active Sensor", "Sensor #%d", currentSensorIndex + 1);
		}
		telemetry.addLine();
		
		telemetry.addLine("Raw RGB Values:");
		telemetry.addData("  Red", "%.1f", rgb[0]);
		telemetry.addData("  Green", "%.1f", rgb[1]);
		telemetry.addData("  Blue", "%.1f", rgb[2]);
		telemetry.addLine();
		
		telemetry.addLine("Distance Calculations:");
		telemetry.addData("  To Green Target", "%.2f", greenDist);
		telemetry.addData("  To Purple Target", "%.2f", purpleDist);
		telemetry.addLine();
		
		// Detection result with confidence check
		boolean greenConfident = greenDist < purpleDist && greenDist < adjustedConfidenceThreshold;
		boolean purpleConfident = purpleDist < greenDist && purpleDist < adjustedConfidenceThreshold;
		
		telemetry.addLine("Detection Result:");
		if (greenConfident) {
			telemetry.addData("  Detected", "✓ GREEN (confident)");
			telemetry.addData("  Confidence", "%.1f below threshold",
					adjustedConfidenceThreshold - greenDist);
		} else if (purpleConfident) {
			telemetry.addData("  Detected", "✓ PURPLE (confident)");
			telemetry.addData("  Confidence", "%.1f below threshold",
					adjustedConfidenceThreshold - purpleDist);
		} else {
			telemetry.addData("  Detected", "✗ NONE (uncertain)");
			double minDist = Math.min(greenDist, purpleDist);
			if (minDist >= adjustedConfidenceThreshold) {
				telemetry.addData("  Reason", "Distance %.1f exceeds threshold %.1f",
						minDist, adjustedConfidenceThreshold);
			} else {
				telemetry.addData("  Reason", "Distances too close (G:%.1f, P:%.1f)",
						greenDist, purpleDist);
			}
		}
		telemetry.addLine();
		
		telemetry.addLine("=== CONFIGURATION ===");
		telemetry.addData("Green Target", "RGB(%.0f, %.0f, %.0f)",
				GREEN_TARGET[0], GREEN_TARGET[1], GREEN_TARGET[2]);
		telemetry.addData("Purple Target", "RGB(%.0f, %.0f, %.0f)",
				PURPLE_TARGET[0], PURPLE_TARGET[1], PURPLE_TARGET[2]);
		telemetry.addData("Default Threshold", "%.1f", CONFIDENCE_THRESHOLD);
		telemetry.addData("Adjusted Threshold", "%.1f (DPAD UP/DOWN)", adjustedConfidenceThreshold);
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
						String.format("%s | RGB(%.0f,%.0f,%.0f) | G:%.1f P:%.1f",
								sample.detected.color,
								sample.rgb[0], sample.rgb[1], sample.rgb[2],
								sample.greenDistance, sample.purpleDistance));
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
			double avgGreenDist = greenSamples.stream()
					.mapToDouble(s -> s.greenDistance)
					.average()
					.orElse(0);
			double minGreenDist = greenSamples.stream()
					.mapToDouble(s -> s.greenDistance)
					.min()
					.orElse(0);
			double maxGreenDist = greenSamples.stream()
					.mapToDouble(s -> s.greenDistance)
					.max()
					.orElse(0);
			telemetry.addData("  GREEN", "%d samples | Avg: %.1f | Range: [%.1f, %.1f]",
					greenSamples.size(), avgGreenDist, minGreenDist, maxGreenDist);
		}
		
		if (!purpleSamples.isEmpty()) {
			double avgPurpleDist = purpleSamples.stream()
					.mapToDouble(s -> s.purpleDistance)
					.average()
					.orElse(0);
			double minPurpleDist = purpleSamples.stream()
					.mapToDouble(s -> s.purpleDistance)
					.min()
					.orElse(0);
			double maxPurpleDist = purpleSamples.stream()
					.mapToDouble(s -> s.purpleDistance)
					.max()
					.orElse(0);
			telemetry.addData("  PURPLE", "%d samples | Avg: %.1f | Range: [%.1f, %.1f]",
					purpleSamples.size(), avgPurpleDist, minPurpleDist, maxPurpleDist);
		}
		
		if (!noneSamples.isEmpty()) {
			telemetry.addData("  NONE", "%d samples (uncertain detections)", noneSamples.size());
		}
	}
	
	private static class ColorSample {
		final double[] rgb;
		final double greenDistance;
		final double purpleDistance;
		final Artifact detected;
		
		ColorSample(double[] rgb, double greenDistance, double purpleDistance, Artifact detected) {
			this.rgb = rgb;
			this.greenDistance = greenDistance;
			this.purpleDistance = purpleDistance;
			this.detected = detected;
		}
	}
}
