package org.firstinspires.ftc.teamcode;

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
		wizard = new MatchConfigurationWizard(matchSettings, gamepad1, logging);
		
		// Initialize robot mechanisms
		mechanisms = new MechanismManager(hardwareMap, matchSettings);
	}
	
	/**
	 * Runs repeatedly after INIT is pressed and before START is pressed.
	 */
	@Override
	public void init_loop() {
		// Allow driver to select match settings using the wizard
		wizard.refresh();
		
		// Draw the initial pose of the robot
		logging.drawDebug(mechanisms.drivetrain.follower);
	}
	
	/**
	 * Runs once, when the driver presses PLAY after having pressed INIT and
	 * configured the robot.
	 */
	@Override
	public void start() {
		// Set up the drivetrain at the correct starting position
		mechanisms.drivetrain.follower.setStartingPose(matchSettings.getAutonomousStartingPose());
		
		// Initialize all mechanisms
		mechanisms.init();
		
		// Build the autonomous sequence based on configuration
		// This is where the magic happens - the path registry automatically
		// handles alliance mirroring, and the sequence builder creates the
		// entire autonomous routine declaratively
		pathRegistry = new PathRegistry(mechanisms.drivetrain.follower, matchSettings);
		
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
		// Update all mechanisms (sensors, motors, etc.)
		mechanisms.update();
		
		// Update the autonomous sequence
		// This single line replaces the entire state machine logic!
		autonomousSequence.update(mechanisms);
		
		// Telemetry - much cleaner than before
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
	 * Comprehensive telemetry logging.
	 */
	private void logTelemetry() {
		// Debug visualization
		logging.drawDebug(mechanisms.drivetrain.follower);
		
		// Configuration info
		logging.addData("Alliance", matchSettings.getAllianceColor());
		logging.addData("Starting Position", matchSettings.getAutoStartingPosition());
		logging.addData("Initial Position", matchSettings.getAutonomousStartingPose());
		
		// Sequence progress
		logging.addLine("");
		logging.addLine("=== SEQUENCE PROGRESS ===");
		logging.addData("Current Action", autonomousSequence.getCurrentActionName());
		logging.addData("Action",
				(autonomousSequence.getCurrentActionIndex() + 1) + " / " + autonomousSequence.getTotalActions());
		logging.addData("Progress", String.format("%.1f%%", autonomousSequence.getProgressPercent()));
		
		// Robot state
		logging.addLine("");
		logging.addLine("=== ROBOT STATE ===");
		logging.addData("Current Position", mechanisms.drivetrain.follower.getPose());
		logging.addNumber("Heading (deg)", Math.toDegrees(mechanisms.drivetrain.follower.getPose().getHeading()));
		
		if (mechanisms.drivetrain.follower.isBusy()) {
			logging.addData("Target Position", mechanisms.drivetrain.follower.getCurrentPath().endPose());
		}
		
		// Timing
		logging.addLine("");
		logging.addNumber("Elapsed Time (s)", opmodeTimer.getElapsedTimeSeconds());
		
		// Update the display
		logging.update();
	}
}
