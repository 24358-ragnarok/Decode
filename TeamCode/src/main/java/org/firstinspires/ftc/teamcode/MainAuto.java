package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRuntime;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousSequence;
import org.firstinspires.ftc.teamcode.configuration.MatchConfigurationWizard;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.hardware.BallSwap;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

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
@Photon
@Autonomous(name = "Run: The Boonstra Special", group = ".Competition", preselectTeleOp = "Run: RAGNAROK")
public class MainAuto extends OpMode {
	
	private Timer opmodeTimer;
	private MatchConfigurationWizard wizard;
	private MechanismManager mechanisms;
	private UnifiedLogging logging;
	// The new optimal structure
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
		MatchState.reset();
		
		wizard = new MatchConfigurationWizard(gamepad1, logging);
		
		// Initialize robot mechanisms
		mechanisms = new MechanismManager(hardwareMap);
		
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
		logging.drawRobot(MatchState.getAutonomousStartingPose());
		
		// Allow driver to select match settings using the wizard
		wizard.refresh();
		mechanisms.setHubColors(
				MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE ? MechanismManager.PresetColor.BLUE
						: MechanismManager.PresetColor.RED);
		
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
		
		mechanisms.ifValid(mechanisms.get(VerticalWheelTransfer.class), transfer -> {
			transfer.setUpForAuto();
		});
		mechanisms.ifValid(mechanisms.get(BallSwap.class), ballSwap -> {
			ballSwap.storeArtifact(new Artifact(Artifact.Color.GREEN, 0, true));
		});
		
		mechanisms.drivetrain.follower.setStartingPose(MatchState.getAutonomousStartingPose());
		// Build the autonomous sequence based on configuration
		// The runtime system allows hot-swapping between different strategies
		// while the PathAction system handles alliance mirroring automatically
		
		AutonomousRuntime runtime = MatchState.getAutonomousRuntime();
		if (MatchState.getAutoStartingPosition() == MatchState.AutoStartingPosition.FAR) {
			autonomousSequence = runtime.buildFarSequence();
		} else {
			autonomousSequence = runtime.buildCloseSequence();
		}
		
		// Start the sequence
		autonomousSequence.start(mechanisms);
		
		// Start the opmode timer
		opmodeTimer.resetTimer();
		setupLogging();
	}
	
	/**
	 * Runs repeatedly during the OpMode.
	 */
	@Override
	public void loop() {
		final UnifiedLogging log = logging;
		final MechanismManager mech = mechanisms;
		log.clearDynamic();
		
		// Update all mechanisms (sensors, motors, etc.)
		mech.update();
		// Update the autonomous sequence
		// This single line replaces the entire state machine logic!
		final AutonomousSequence sequence = autonomousSequence;
		if (sequence != null) {
			sequence.update(mech);
		}
		
		// Clear dynamic telemetry and log updated data
		log.drawDebug(mech.drivetrain.follower);
		log.update();
	}
	
	/**
	 * Runs when "stop" is pressed on the Driver Station.
	 * Cleanup and shutdown should occur instantaneously and be non-blocking.
	 */
	@Override
	public void stop() {
		// Store the actual robot pose for TeleOp to use as starting position
		if (mechanisms != null && mechanisms.drivetrain.follower != null) {
			MatchState.setStoredPose(mechanisms.drivetrain.follower.getPose());
			mechanisms.stop();
		}
		
		if (autonomousSequence != null) {
			autonomousSequence.stop(mechanisms);
		}
	}
	
	/**
	 * Comprehensive telemetry logging - optimized with lazy evaluation.
	 */
	private void setupLogging() {
		logging.addDataLazy("Classifier", MatchState::getClassifier);
		logging.addDataLazy("Current Position", () -> mechanisms.drivetrain.follower.getPose());
		logging.addDataLazy("Current Action", () -> autonomousSequence.getCurrentActionName());
		logging.addDataLazy("Action", () -> {
			int current = autonomousSequence.getCurrentActionIndex() + 1;
			int total = autonomousSequence.getTotalActions();
			double percent = autonomousSequence.getProgressPercent();
			return String.format("%d / %d (%.1f%%)", current, total, percent);
		});
		
		logging.addDataLazy("Path Beginning Position", () -> {
			if (mechanisms.drivetrain.follower.isBusy()) {
				return mechanisms.drivetrain.follower.getCurrentPath().getPoseInformation(0).getPose().toString();
			} else {
				return "N/A";
			}
		});
		logging.addDataLazy("Path End Position", () -> {
			if (mechanisms.drivetrain.follower.isBusy()) {
				return mechanisms.drivetrain.follower.getCurrentPath().endPose().toString();
			} else {
				return "N/A";
			}
		});
		
		logging.addDataLazy("Elapsed Time", "%.2f",
				() -> opmodeTimer.getElapsedTimeSeconds());
		
	}
}
