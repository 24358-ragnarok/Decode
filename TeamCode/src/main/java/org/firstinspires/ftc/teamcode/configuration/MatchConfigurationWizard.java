package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * MatchConfigurationWizard provides an interface for configuring match settings
 * during the init_loop phase of autonomous programs.
 */
@SuppressWarnings({"MethodWithMultipleReturnPoints", "ClassWithoutNoArgConstructor", "ClassHasNoToStringMethod", "HardcodedFileSeparator"})
public class MatchConfigurationWizard {
	private final Gamepad gamepad1;
	private final UnifiedLogging logging;
	private boolean confirmed = false;

	/**
	 * Creates a new MatchConfigurationWizard
	 *
	 * @param gamepad1 The primary gamepad for input
	 * @param logging  Telemetry instance for displaying current settings
	 */
	public MatchConfigurationWizard(Gamepad gamepad1, UnifiedLogging logging) {
		this.gamepad1 = gamepad1;
		this.logging = logging;
	}

	/**
	 * Call this method repeatedly in init_loop to process input and update display
	 */
	public void refresh() {
		// If settings are confirmed, only allow START to unlock; ignore other inputs
		if (confirmed) {
			if (gamepad1.startWasPressed()) {
				confirmed = false;
			}
			updateTelemetry();
			return;
		}

		// Detect rising edge of dpad_up (just pressed)
		if (gamepad1.bWasPressed()) {
			MatchState.setAllianceColor(MatchState.AllianceColor.RED);
		}

		// Detect rising edge of dpad_down (just pressed)
		if (gamepad1.xWasPressed()) {
			MatchState.setAllianceColor(MatchState.AllianceColor.BLUE);
		}

		if (gamepad1.aWasPressed()) {
			setStartingPositionWithRuntimeCheck(MatchState.AutoStartingPosition.CLOSE);
		}
		
		if (gamepad1.yWasPressed()) {
			setStartingPositionWithRuntimeCheck(MatchState.AutoStartingPosition.FAR);
		}
		
		if (gamepad1.startWasPressed()) {
			confirmed = !confirmed;
		}
		
		// Cycle through runtimes with left/right d-pad (only to compatible ones)
		if (gamepad1.dpadRightWasPressed()) {
			MatchState.nextAutonomousRuntimeFor(MatchState.getAutoStartingPosition());
		}
		
		if (gamepad1.dpadLeftWasPressed()) {
			MatchState.previousAutonomousRuntimeFor(MatchState.getAutoStartingPosition());
		}
		
		// Update telemetry display
		updateTelemetry();
	}
	
	/**
	 * Sets the starting position, switching runtime if the current one doesn't
	 * support it.
	 */
	private void setStartingPositionWithRuntimeCheck(MatchState.AutoStartingPosition newPosition) {
		MatchState.setAutoStartingPosition(newPosition);
		
		// If current runtime doesn't support the new position, switch to one that does
		if (!MatchState.getAutonomousRuntime().supportsPosition(newPosition)) {
			MatchState.nextAutonomousRuntimeFor(newPosition);
		}
	}
	
	/**
	 * Updates telemetry with current configuration and instructions
	 */
	private void updateTelemetry() {
		MatchState.AllianceColor currentColor = MatchState.getAllianceColor();
		MatchState.AutoStartingPosition autoStartingPosition = MatchState.getAutoStartingPosition();
		
		if (!confirmed) {
			logging.addLine("=== MATCH CONFIGURATION ===");
			logging.addLine("  X/B           → Alliance Color");
			logging.addLine("  A/Y           → Starting Position");
			logging.addLine("  D-Pad L/R     → Runtime");
			logging.addLine("  START         → Confirm");
		} else {
			logging.addLine("=== CONFIGURATION CONFIRMED ===");
			logging.addLine("press start to go back");
		}
		
		logging.addLine("");
		
		logging.addLine(String.format("Starting Conditions: %s %s",
				(currentColor == MatchState.AllianceColor.BLUE) ? "Blue" : "Red",
				(autoStartingPosition == MatchState.AutoStartingPosition.CLOSE) ? "Close" : "Far"));
		
		logging.addLine("Runtime: " + MatchState.getAutonomousRuntime().getDisplayName());
		logging.addLine("Starting Pos: " + MatchState.getAutonomousStartingPose().toString());
	}
}
