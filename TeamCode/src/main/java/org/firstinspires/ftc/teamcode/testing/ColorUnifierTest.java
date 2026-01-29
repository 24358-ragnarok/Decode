package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.ColorRangefinder;
import org.firstinspires.ftc.teamcode.software.ColorSensor;
import org.firstinspires.ftc.teamcode.software.ColorUnifier;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

import java.util.ArrayList;
import java.util.List;

/**
 * Test OpMode for ColorUnifier with detection history and mechanism controls.
 * <p>
 * Displays real-time color detection results with a history of detections,
 * showing when each detection occurred (time and transfer ticks). Useful for
 * tuning debounce timing and verifying detection accuracy.
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li>X: Crawl intake (hold)</li>
 * <li>B: Advance transfer</li>
 * <li>A: Reverse transfer</li>
 * <li>Y: Clear detection history</li>
 * </ul>
 * <p>
 * <b>FEATURES:</b>
 * <ul>
 * <li>Real-time detection status</li>
 * <li>Detection history with timestamps and transfer ticks</li>
 * <li>Debounce timer display</li>
 * <li>Intake and transfer wheel controls</li>
 * </ul>
 */
@TeleOp(name = "Test: Color Unifier", group = "Tests")
public class ColorUnifierTest extends OpMode {
	private static final int MAX_HISTORY_SIZE = 20;
	private final List<DetectionEntry> detectionHistory = new ArrayList<>();
	private MechanismManager mechanisms;
	private ColorUnifier colorUnifier;
	private VerticalWheelTransfer transfer;
	private FlexVectorIntake intake;
	private Artifact lastDetected = Artifact.NONE;
	private double startTime;
	
	@Override
	public void init() {
		mechanisms = new MechanismManager(hardwareMap);
		transfer = mechanisms.get(VerticalWheelTransfer.class);
		intake = mechanisms.get(FlexVectorIntake.class);
		
		// Build color sensors using same approach as MechanismManager
		ColorSensor[] colorSensors = buildColorSensors();
		
		// Initialize ColorUnifier (no rangefinders)
		colorUnifier = new ColorUnifier(mechanisms, new ColorRangefinder[0], colorSensors);
		
		startTime = System.currentTimeMillis();
		
		telemetry.addLine("Color Unifier Test Ready");
		telemetry.addLine();
		telemetry.addLine("Controls:");
		telemetry.addLine("  X: Intake (hold)");
		telemetry.addLine("  B: Advance transfer");
		telemetry.addLine("  A: Reverse transfer");
		telemetry.addLine("  Y: Clear history");
		telemetry.addLine();
		telemetry.addLine("Debounce Time: " + Settings.Intake.COLOR_SENSOR_DEBOUNCE_TIME_MS + " ms");
		telemetry.update();
	}
	
	@Override
	public void loop() {
		mechanisms.update(); // Required for mechanisms to apply motor commands
		
		// Handle gamepad controls
		if (gamepad1.x) {
			if (intake != null) {
				intake.in();
			}
		} else if (gamepad1.xWasReleased()) {
			if (intake != null) {
				intake.stop();
			}
		}
		
		if (gamepad1.bWasPressed()) {
			if (transfer != null) {
				transfer.advance();
			}
		}
		
		if (gamepad1.aWasPressed()) {
			if (transfer != null) {
				transfer.reverse();
			}
		}
		
		if (gamepad1.yWasPressed()) {
			detectionHistory.clear();
		}
		
		// Get current detection
		Artifact detected = colorUnifier.find();
		double currentTime = System.currentTimeMillis() - startTime;
		double currentTicks = transfer != null ? transfer.getTicks() : 0;
		
		// Add to history if new detection
		if (detected != Artifact.NONE && detected.color != lastDetected.color) {
			detectionHistory.add(new DetectionEntry(detected.color, currentTime, currentTicks));
			// Keep history size manageable
			if (detectionHistory.size() > MAX_HISTORY_SIZE) {
				detectionHistory.remove(0);
			}
		}
		lastDetected = detected;
		
		// Display telemetry
		displayTelemetry(currentTime, currentTicks);
	}
	
	private void displayTelemetry(double currentTime, double currentTicks) {
		telemetry.addLine("=== CURRENT STATUS ===");
		Artifact current = colorUnifier.find();
		telemetry.addData("Detected Color", current.color);
		telemetry.addData("Time Since Start", "%.1f ms", currentTime);
		if (transfer != null) {
			telemetry.addData("Transfer Ticks", "%.1f", currentTicks);
			telemetry.addData("Transfer Busy", transfer.isBusy());
		}
		telemetry.addLine();
		
		telemetry.addLine("=== DEBOUNCE INFO ===");
		telemetry.addData("Debounce Time", Settings.Intake.COLOR_SENSOR_DEBOUNCE_TIME_MS + " ms");
		telemetry.addLine();
		
		telemetry.addLine("=== DETECTION HISTORY ===");
		if (detectionHistory.isEmpty()) {
			telemetry.addLine("(No detections yet)");
		} else {
			// Show most recent detections first
			int startIdx = Math.max(0, detectionHistory.size() - 10);
			for (int i = detectionHistory.size() - 1; i >= startIdx; i--) {
				DetectionEntry entry = detectionHistory.get(i);
				telemetry.addData(
						String.format("#%d", i + 1),
						String.format("%s at %.1f ms (%.1f ticks)",
								entry.color, entry.timeMs, entry.transferTicks));
			}
			if (detectionHistory.size() > 10) {
				telemetry.addLine("... (" + (detectionHistory.size() - 10) + " more)");
			}
		}
		telemetry.addLine();
		
		telemetry.addLine("=== MECHANISM STATUS ===");
		if (intake != null) {
			telemetry.addData("Intake", intake.state != FlexVectorIntake.IntakeState.STOPPED ? "RUNNING" : "STOPPED");
		}
		if (transfer != null) {
			telemetry.addData("Transfer", transfer.isBusy() ? "BUSY" : "IDLE");
		}
		
		telemetry.update();
	}
	
	/**
	 * Builds ColorSensor array from hardware map using same approach as
	 * MechanismManager.
	 * Returns empty array if no color sensors are configured.
	 */
	private ColorSensor[] buildColorSensors() {
		List<ColorSensor> list = new ArrayList<>();
		
		try {
			list.add(new ColorSensor(Settings.Hardware.COLOR_SENSOR_LEFT.fromHardwareMap(hardwareMap)));
		} catch (Exception e) {
			// Color sensor not configured, skip
		}
		
		try {
			list.add(new ColorSensor(Settings.Hardware.COLOR_SENSOR_RIGHT.fromHardwareMap(hardwareMap)));
		} catch (Exception e) {
			// Color sensor not configured, skip
		}
		
		// Initialize all sensors
		for (ColorSensor sensor : list) {
			sensor.init();
		}
		
		return list.toArray(new ColorSensor[0]);
	}
	
	private static class DetectionEntry {
		final Artifact.Color color;
		final double timeMs;
		final double transferTicks;
		
		DetectionEntry(Artifact.Color color, double timeMs, double transferTicks) {
			this.color = color;
			this.timeMs = timeMs;
			this.transferTicks = transferTicks;
		}
	}
}
