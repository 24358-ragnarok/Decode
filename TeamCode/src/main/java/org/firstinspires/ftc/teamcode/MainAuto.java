package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MainOp.ifMechanismValid;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousSequence;
import org.firstinspires.ftc.teamcode.autonomous.PathRegistry;
import org.firstinspires.ftc.teamcode.autonomous.SequenceBuilder;
import org.firstinspires.ftc.teamcode.configuration.MatchConfigurationWizard;
import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

/**
 * The main Autonomous script that makes the robot move by itself during the
 * Auto period of a match.
 * <p>
 * This implementation uses an optimal action-based architecture that provides:
 * - Declarative sequence definitions for readability
 * - Automatic path mirroring (no red/blue duplication)
 * - Type-safe state management with actions
 * - Easy modification and testing of individual segments
 * - Graceful error handling when mechanisms are unavailable
 * <p>
 * The old state machine approach (600+ lines, lots of duplication) has been
 * replaced with a clean, maintainable structure (~150 lines).
 */
@Autonomous(name = "Main Auto", group = ".Competition Modes")
public class MainAuto extends OpMode {

	private Timer opmodeTimer;
	private MatchConfigurationWizard wizard;
	private MechanismManager mechanisms;
	private MatchSettings matchSettings;
	private UnifiedLogging logging;

	// The new optimal structure
	private PathRegistry pathRegistry;
	private AutonomousSequence autonomousSequence;
	
	/**
	 * Runs when INIT is pressed on the driver station.
	 */
	@Override
	public void init() {
		// Create fresh timer and logging
		opmodeTimer = new Timer();
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		
		// Match settings will be configured by the driver during init_loop
		matchSettings = new MatchSettings(blackboard);
		
		// Initialize blackboard with default values to ensure clean state
		// This prevents stale data from previous runs from affecting the current run
		matchSettings.setAllianceColor(MatchSettings.AllianceColor.BLUE);
		matchSettings.setAutoStartingPosition(MatchSettings.AutoStartingPosition.CLOSE);
		
		wizard = new MatchConfigurationWizard(matchSettings, gamepad1, logging);
		
		// Initialize robot mechanisms
		mechanisms = new MechanismManager(hardwareMap, matchSettings);
		
		// Enable retained mode for efficient telemetry during autonomous
		logging.enableRetainedMode();
	}
	
	/**
	 * Runs repeatedly after INIT is pressed and before START is pressed.
	 */
	@Override
	public void init_loop() {
		logging.clearDynamic();
		// Draw the initial pose of the robot
		logging.drawRobot(matchSettings.getAutonomousStartingPose());
		
		// Allow driver to select match settings using the wizard
		wizard.refresh();
		
		logging.update();
	}
	
	/**
	 * Runs once, when the driver presses PLAY after having pressed INIT and
	 * configured the robot.
	 */
	@Override
	public void start() {
		// Initialize all mechanisms
		mechanisms.start();
		
		ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), swt -> {
			swt.slots[0] = MatchSettings.ArtifactColor.PURPLE;
			swt.slots[1] = MatchSettings.ArtifactColor.PURPLE;
			swt.slots[2] = MatchSettings.ArtifactColor.PURPLE;
		});
		
		mechanisms.drivetrain.follower.setStartingPose(matchSettings.getAutonomousStartingPose());
		// Build the autonomous sequence based on configuration
		// This is where the magic happens - the path registry automatically
		// handles alliance mirroring, and the sequence builder creates the
		// entire autonomous routine declaratively
		
		pathRegistry = new PathRegistry(mechanisms.drivetrain.follower,
				matchSettings);
		
		if (matchSettings.getAutoStartingPosition() == MatchSettings.AutoStartingPosition.FAR) {
			autonomousSequence = SequenceBuilder.buildFarSequence(pathRegistry);
		} else {
			autonomousSequence = SequenceBuilder.buildCloseSequence(pathRegistry);
		}
		
		// Start the sequence
		autonomousSequence.start(mechanisms);
		
		// Start the opmode timer
		opmodeTimer.resetTimer();
	}
	
	/**
	 * Runs repeatedly during the OpMode.
	 */
	@Override
	public void loop() {
		mechanisms.drivetrain.follower.update();
		
		// Update all mechanisms (sensors, motors, etc.)
		mechanisms.update();
		
		// Update the autonomous sequence
		// This single line replaces the entire state machine logic!
		autonomousSequence.update(mechanisms);
		
		// Clear dynamic telemetry and log updated data
		logging.clearDynamic();
		logTelemetry();
	}
	
	/**
	 * Runs when "stop" is pressed on the Driver Station.
	 * Cleanup and shutdown should occur instantaneously and be non-blocking.
	 */
	@Override
	public void stop() {
		if (autonomousSequence != null) {
			autonomousSequence.stop(mechanisms);
		}
		mechanisms.stop();
	}
	
	/**
	 * Comprehensive telemetry logging - optimized with lazy evaluation.
	 */
	private void logTelemetry() {
		// Debug visualization
		logging.drawDebug(mechanisms.drivetrain.follower);
		
		// Configuration info (static, retained)
		logging.addData("Alliance", matchSettings.getAllianceColor());
		logging.addData("Starting Position", matchSettings.getAutoStartingPosition());
		logging.addData("Initial Position", matchSettings.getAutonomousStartingPose());
		
		// Sequence progress - use lazy evaluation to avoid unnecessary string
		// operations
		logging.addLine("");
		logging.addLine("=== SEQUENCE PROGRESS ===");
		logging.addDataLazy("Current Action", () -> autonomousSequence.getCurrentActionName());
		logging.addDataLazy("Action",
				() -> (autonomousSequence.getCurrentActionIndex() + 1) + " / " +
						autonomousSequence.getTotalActions());
		logging.addDataLazy("Progress", "%.1f%%",
				() -> autonomousSequence.getProgressPercent());
		
		// Robot state - use lazy evaluation for pose calculations
		logging.addLine("");
		logging.addLine("=== ROBOT STATE ===");
		logging.addDataLazy("Current Position", () -> mechanisms.drivetrain.follower.getPose());
		logging.addDataLazy("Heading (deg)", "%.2f",
				() -> Math.toDegrees(mechanisms.drivetrain.follower.getPose().getHeading()));
		
		// Conditional telemetry for path info (dynamic)
		if (mechanisms.drivetrain.follower.isBusy()) {
			logging.addDataLazy("Path End Position",
					() -> mechanisms.drivetrain.follower.getCurrentPath().endPose());
			logging.addDataLazy("Path Beginning Position",
					() -> mechanisms.drivetrain.follower.getCurrentPath().getPoseInformation(0).getPose());
		}
		
		// Timing - use lazy evaluation
		logging.addLine("");
		logging.addDataLazy("Elapsed Time (s)", "%.2f",
				() -> opmodeTimer.getElapsedTimeSeconds());
		
		// Update the display
		logging.update();
	}
}
