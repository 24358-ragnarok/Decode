package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * UnifiedLogging allows us to send data to both the Driver Station and the
 * Panels Webview
 * simultaneously, and handles all the messy stuff like formatting and Field
 * Drawings.
 * <p>
 * Optimizations:
 * - Uses retained telemetry items (setAutoClear(false)) to avoid rebuilding
 * each frame
 * - Caches Telemetry.Item references for efficient updates
 * - Uses Func suppliers for expensive operations that are only evaluated when
 * transmission occurs
 * - Optimized transmission interval to reduce overhead
 */
public class UnifiedLogging {
	public final Telemetry driverStation;
	public final TelemetryManager panels;
	
	// Cache telemetry items for efficient updates
	private final Map<String, Telemetry.Item> itemCache = new HashMap<>();
	private final Map<String, Telemetry.Line> lineCache = new HashMap<>();
	
	// Track items that should be cleared each frame (non-retained)
	private boolean retainedMode = false;
	
	public UnifiedLogging(Telemetry telemetry, TelemetryManager panels) {
		this.driverStation = telemetry;
		this.panels = panels;
		Drawing.init();
		
		// Optimize telemetry settings
		driverStation.setAutoClear(false); // We'll manage clearing manually
		driverStation.setMsTransmissionInterval(100); // Update every 100ms (was default 250ms)
	}
	
	/**
	 * Enable retained mode - items persist across update() calls.
	 * Call this once during init to set up persistent telemetry items.
	 */
	public void enableRetainedMode() {
		retainedMode = true;
	}
	
	/**
	 * Adds a line of text that will be recreated each frame.
	 */
	public void addLine(String line) {
		driverStation.addLine(line);
		panels.addLine(line);
	}
	
	/**
	 * Adds a labeled line of text with a cached reference for efficient updates.
	 */
	public Telemetry.Line addLabeledLine(String label) {
		Telemetry.Line line = lineCache.get(label);
		if (line == null) {
			line = driverStation.addLine(label);
			lineCache.put(label, line);
		}
		panels.addLine(label);
		return line;
	}
	
	/**
	 * Adds or updates a data item. Uses cached items in retained mode for
	 * efficiency.
	 */
	public void addData(String key, Object value) {
		Telemetry.Item item = itemCache.get(key);
		if (item == null) {
			item = driverStation.addData(key, value);
			if (retainedMode) {
				item.setRetained(true);
				itemCache.put(key, item);
			}
		} else {
			item.setValue(value);
		}
		panels.addData(key, value);
	}
	
	/**
	 * Adds or updates a formatted number. Uses cached items for efficiency.
	 */
	public void addNumber(String key, double value) {
		Telemetry.Item item = itemCache.get(key);
		if (item == null) {
			item = driverStation.addData(key, "%.2f", value);
			if (retainedMode) {
				item.setRetained(true);
				itemCache.put(key, item);
			}
		} else {
			item.setValue("%.2f", value);
		}
		panels.addData(key, String.format(Locale.US, "%.2f", value));
	}
	
	/**
	 * Adds data with a custom format string.
	 */
	public void addData(String key, String format, Object... args) {
		Telemetry.Item item = itemCache.get(key);
		if (item == null) {
			item = driverStation.addData(key, format, args);
			if (retainedMode) {
				item.setRetained(true);
				itemCache.put(key, item);
			}
		} else {
			item.setValue(format, args);
		}
		panels.addData(key, String.format(Locale.US, format, args));
	}
	
	/**
	 * Adds data using a Func supplier - only evaluated when telemetry is
	 * transmitted.
	 * This is ideal for expensive operations like pose calculations.
	 */
	public <T> void addDataLazy(String key, Func<T> valueProducer) {
		Telemetry.Item item = itemCache.get(key);
		if (item == null) {
			item = driverStation.addData(key, valueProducer);
			item.setRetained(true); // Func items are always retained
			itemCache.put(key, item);
		} else {
			// Update the item with new func (requires recreating)
			driverStation.removeItem(item);
			item = driverStation.addData(key, valueProducer);
			item.setRetained(true);
			itemCache.put(key, item);
		}
	}
	
	/**
	 * Adds formatted data using a Func supplier with custom formatting.
	 */
	public <T> void addDataLazy(String key, String format, Func<T> valueProducer) {
		Telemetry.Item item = itemCache.get(key);
		if (item == null) {
			item = driverStation.addData(key, format, valueProducer);
			item.setRetained(true);
			itemCache.put(key, item);
		} else {
			// Update the item with new func (requires recreating)
			driverStation.removeItem(item);
			item = driverStation.addData(key, format, valueProducer);
			item.setRetained(true);
			itemCache.put(key, item);
		}
	}
	
	/**
	 * Clears only non-retained items. Call this at the start of each loop
	 * if you want to clear dynamic data while keeping retained items.
	 */
	public void clearDynamic() {
		driverStation.clear(); // Only removes non-retained items
	}
	
	/**
	 * Clears ALL items including retained ones. Use sparingly.
	 */
	public void clearAll() {
		driverStation.clearAll();
		itemCache.clear();
		lineCache.clear();
	}
	
	public void drawRobot(Pose pose) {
		Drawing.drawRobot(pose);
	}
	
	public void drawDebug(Follower follower) {
		Drawing.drawDebug(follower);
	}
	
	public void drawPath(PathChain path) {
		Drawing.drawPath(path);
	}
	
	public void update() {
		Drawing.update();
		panels.update();
		driverStation.update();
	}
}
