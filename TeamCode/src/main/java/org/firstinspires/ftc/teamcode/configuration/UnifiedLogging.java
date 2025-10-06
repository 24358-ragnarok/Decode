package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.Locale;

/**
 * UnifiedLogging allows us to send data to both the Driver Station and the Panels Webview
 * simultaneously, and handles all the messy stuff like formatting and Field Drawings.
 */
public class UnifiedLogging {
	public final Telemetry driverStation;
	public final TelemetryManager panels;
	
	public UnifiedLogging(Telemetry telemetry, TelemetryManager panels) {
		this.driverStation = telemetry;
		this.panels = panels;
		Drawing.init();
	}
	
	public void addLine(String line) {
		driverStation.addLine(line);
		panels.addLine(line);
	}
	
	public void addData(String key, Object value) {
		driverStation.addData(key, value);
		panels.addData(key, value);
	}
	
	public void addNumber(String key, double value) {
		driverStation.addData(key, "%.2f", value);
		panels.addData(key, String.format(Locale.US, "%.2f", value));
	}
	
	public void drawRobot(Pose pose) {
		Drawing.drawRobot(pose);
	}
	
	public void drawDebug(Follower follower) {
		Drawing.drawDebug(follower);
		panels.addData("panels first drawing point", follower.getCurrentPath().getPanelsDrawingPoints()[0][0]);
	}
	
	public void drawPath(PathChain path) {
		Drawing.drawPath(path, Drawing.targetLook);
	}
	
	public void update() {
		Drawing.update();
		panels.update();
		driverStation.update();
	}
}
