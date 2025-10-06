package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * MatchConfigurationWizard provides an interface for configuring match settings
 * during the init_loop phase of autonomous programs.
 * <p>
 * Uses gamepad1 d-pad up/down to toggle between RED and BLUE alliance colors.
 */
public class MatchConfigurationWizard {
	private final MatchSettings matchSettings;
	private final Gamepad gamepad1;
	private final UnifiedLogging logging;
	private boolean confirmed = false;
	
	/**
	 * Creates a new MatchConfigurationWizard
	 *
	 * @param matchSettings The MatchSettings instance to modify
	 * @param gamepad1      The primary gamepad for input
	 * @param logging       Telemetry instance for displaying current settings
	 */
	public MatchConfigurationWizard(MatchSettings matchSettings, Gamepad gamepad1, UnifiedLogging logging) {
		this.matchSettings = matchSettings;
		this.gamepad1 = gamepad1;
		this.logging = logging;
	}
	
	/**
	 * Call this method repeatedly in init_loop to process input and update display
	 */
	public void refresh() {
		// Detect rising edge of dpad_up (just pressed)
		if (gamepad1.bWasPressed()) {
			matchSettings.setAllianceColor(MatchSettings.AllianceColor.RED);
		}
		
		// Detect rising edge of dpad_down (just pressed)
		if (gamepad1.xWasPressed()) {
			matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE);
		}
		
		if (gamepad1.aWasPressed()) {
			matchSettings.setAutoStartingPosition(MatchSettings.AutoStartingPosition.CLOSE);
		}
		
		if (gamepad1.yWasPressed()) {
			matchSettings.setAutoStartingPosition(MatchSettings.AutoStartingPosition.FAR);
		}
		
		if (gamepad1.startWasPressed()) {
			confirmed = !confirmed;
		}
		
		// Update telemetry display
		updateTelemetry();
	}
	
	/**
	 * Updates telemetry with current configuration and instructions
	 */
	private void updateTelemetry() {
		MatchSettings.AllianceColor currentColor = matchSettings.getAllianceColor();
		MatchSettings.AutoStartingPosition autoStartingPosition = matchSettings.getAutoStartingPosition();
		
		if (!confirmed) {
			logging.addLine("=== MATCH CONFIGURATION ===");
			logging.addLine("  X           → BLUE Alliance");
			logging.addLine("  B           → RED Alliance");
			logging.addLine("  A           → Close Starting Position");
			logging.addLine("  Y           → Far Starting Position");
			logging.addLine("  START       → Confirm Configuration");
		} else {
			logging.addLine("=== CONFIGURATION CONFIRMED ===");
			logging.addLine("❎ Press start to cancel");
		}
		
		logging.addLine("");
		
		if (currentColor == MatchSettings.AllianceColor.BLUE) {
			logging.addLine("\uD83D\uDD35 BLUE Alliance Selected");
		} else {
			logging.addLine("\uD83D\uDD34 RED Alliance Selected");
		}
		
		if (autoStartingPosition == MatchSettings.AutoStartingPosition.CLOSE) {
			logging.addLine("\uD83D\uDD0D Close Starting Position Selected");
		} else {
			logging.addLine("\uD83D\uDD2D Far Starting Position Selected");
		}
		
		logging.addData("Starting Pose", matchSettings.getAutonomousStartingPose());
		
		logging.update();
	}
}
