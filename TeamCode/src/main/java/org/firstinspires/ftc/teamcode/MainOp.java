package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.hardware.BallSwap;
import org.firstinspires.ftc.teamcode.hardware.BentDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.PairedLauncher;
import org.firstinspires.ftc.teamcode.hardware.VerticalWheelTransfer;
import org.firstinspires.ftc.teamcode.software.Controller;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

/**
 * This is our main TeleOp class for the driver-controlled period, which occurs
 * after Auto.
 * Handles controller profile selection and robot operation during matches.
 */
@Photon
@TeleOp(name = "Run: RAGNAROK", group = ".Competition")
public class MainOp extends OpMode {
	private final StringBuilder transferSlotsDisplayBuilder = new StringBuilder();
	private boolean reset = false;
	private UnifiedLogging logging;
	private MechanismManager mechanisms;
	private Controller mainController;
	private Controller subController;
	private BentDrivetrain bentDrivetrain;
	private VerticalWheelTransfer transfer;
	private FlexVectorIntake intake;
	private PairedLauncher launcher;
	private BallSwap swap;
	private Timer speedTimer;
	private double speedms = 0;
	
	/**
	 * Runs when "init" is pressed on the Driver Station.
	 * Initializes all robot systems, controllers, and telemetry for TeleOp
	 * operation.
	 */
	@Override
	public final void init() {
		speedTimer = new Timer();
		// Initialize robot systems
		mechanisms = new MechanismManager(hardwareMap);
		// Cache mechanism references once to avoid map lookups in the hot loop
		bentDrivetrain = mechanisms.bentDrivetrain;
		transfer = mechanisms.get(VerticalWheelTransfer.class);
		intake = mechanisms.get(FlexVectorIntake.class);
		launcher = mechanisms.get(PairedLauncher.class);
		swap = mechanisms.get(BallSwap.class);
		mainController = new Controller(gamepad1, mechanisms.bentDrivetrain.follower,
				PanelsGamepad.INSTANCE.getFirstManager());
		subController = new Controller(gamepad2, mechanisms.bentDrivetrain.follower,
				PanelsGamepad.INSTANCE.getSecondManager());
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		mechanisms.bentDrivetrain.follower.setStartingPose(MatchState.getTeleOpStartingPose());
		mechanisms.bentDrivetrain.switchToManual();
		setupLogging();
		// Show whether we're using stored pose or fallback
		Pose storedPose = MatchState.getStoredPose();
		if (storedPose != null) {
			logging.addData("Starting Pose Source", "STORED FROM PREVIOUS");
			logging.addData("Stored Pose", storedPose);
		} else {
			logging.addData("Starting Pose Source", "FALLBACK/PREDEFINED");
		}
		mechanisms.setHubColors(
				MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE ? MechanismManager.PresetColor.BLUE
						: MechanismManager.PresetColor.RED);
	}
	
	/**
	 * Runs after "init" and before "start" repeatedly.
	 */
	@Override
	public final void init_loop() {
		logging.drawDebug(mechanisms.bentDrivetrain.follower);
		logging.update();
	}
	
	/**
	 * Runs when "start" is pressed on the Driver Station.
	 */
	@Override
	public final void start() {
		// Initialize mechanisms and start teleop drive
		mechanisms.ifValid(mechanisms, MechanismManager::start);
		mechanisms.bentDrivetrain.follower.startTeleopDrive(); // or pass in true to enable braking
		MatchState.clearStoredPose();
		logging.enableRetainedMode();
	}
	
	/**
	 * Runs repeatedly after "start" is pressed on the Driver Station, during the
	 * actual game.
	 */
	@Override
	public final void loop() {
		final UnifiedLogging log = logging;
		final MechanismManager mech = mechanisms;
		log.clearDynamic();
		
		mech.update();
		
		processControllerInputs();
		setControllerLEDs();
		setControllerRumble();
		
		// Draw debug visualization (retained items like Heading, X, Y are auto-updated
		// via Func)
		log.drawDebug(mech.bentDrivetrain.follower);
		log.update();
	}
	
	/**
	 * Runs when "stop" is pressed on the Driver Station.
	 * Cleanup and shutdown should occur instantaneously and be non-blocking.
	 */
	@Override
	public final void stop() {
		// Store the actual robot pose for future reference or debugging
		if (mechanisms != null && mechanisms.bentDrivetrain.follower != null) {
			MatchState.setStoredPose(mechanisms.bentDrivetrain.follower.getPose());
			mechanisms.stop();
		}
	}
	
	/**
	 * Process controller inputs for both main and sub controllers.
	 * Handles bentDrivetrain movement, launcher controls, intake operations, and
	 * telemetry updates.
	 */
	private void processControllerInputs() {
		// Refresh gamepad state FIRST (required for edge detection to work)
		mainController.update();
		subController.update();
		
		// Hot-loop locals
		final BentDrivetrain bentDrivetrain = this.bentDrivetrain;
		final VerticalWheelTransfer transfer = this.transfer;
		final FlexVectorIntake intake = this.intake;
		final PairedLauncher launcher = this.launcher;
		final BallSwap swap = this.swap;
		
		// BentDrivetrain
		double d = mainController.getProcessedDrive();
		double s = mainController.getProcessedStrafe();
		double r = mainController.getProcessedRotation();
		
		if (bentDrivetrain != null) {
			if (mainController.wasJustPressed(Controller.Action.TOGGLE_CENTRICITY)) {
				bentDrivetrain.toggleCentricity();
			}
			
			if (mainController.wasJustPressed(Controller.Action.RESET_FOLLOWER)) {
				bentDrivetrain.follower.setPose(
						(MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE)
								? Settings.Positions.Default.RESET
								: Settings.Field.mirrorPose(Settings.Positions.Default.RESET));
			}
			
			bentDrivetrain.manual(d, s, r);
			
			final double startValue = mainController.getProcessedValue(Controller.Control.START);
			for (Controller.Action action : Settings.Controls.gotoActions) {
				if (mainController.wasJustPressed(action) && startValue <= 0.0) {
					bentDrivetrain.goTo(action);
					// Stop intake when going to park
					if (bentDrivetrain.actionToPosition(action) == BentDrivetrain.Position.PARK && transfer != null) {
						transfer.freeze();
					}
				} else if (mainController.wasJustReleased(action)) {
					bentDrivetrain.switchToManual();
				}
			}
		}
		
		// Alignment & Launcher
		if (launcher != null) {
			if (subController.getProcessedValue(Controller.Action.AIM) > 0.1) {
				launcher.ready();
				if (launcher.isAtSpeed() && speedms == 0) {
					speedms = speedTimer.getElapsedTime();
					reset = false;
				}
				if (!launcher.isAtSpeed() && speedms != 0 && !reset) {
					speedms = 0;
					reset = true;
					speedTimer.resetTimer();
				}
				logging.addData(".ms back to speed", speedms);
			} else {
				launcher.stop();
			}
			if (subController.wasJustPressed(Controller.Action.LAUNCH)) {
				launcher.open();
			} else if (subController.wasJustReleased(Controller.Action.LAUNCH)) {
				launcher.close();
			}
		}
		
		if (transfer != null) {
			if (subController.wasJustPressed(Controller.Action.TRANSFER_ADVANCE)) {
				transfer.advance();
			}
			if (subController.wasJustPressed(Controller.Action.TRANSFER_REVERSE)) {
				transfer.reverse();
			}
		}
		
		// Intake & Transfer
		if (intake != null) {
			if (subController.wasJustPressed(Controller.Action.INTAKE_IN)) {
				intake.in();
			}
			if (subController.wasJustReleased(Controller.Action.INTAKE_IN)) {
				intake.stop();
			}
			
			if (subController.wasJustPressed(Controller.Action.INTAKE_OUT)) {
				intake.out();
			}
			if (subController.wasJustReleased(Controller.Action.INTAKE_OUT)) {
				intake.stop();
			}
		}
		
		if (swap != null) {
			if (subController.wasJustPressed(Controller.Control.RIGHT_BUMPER)) {
				swap.grab();
			}
			if (subController.wasJustPressed(Controller.Control.LEFT_BUMPER)) {
				swap.hold();
			}
		}
	}
	
	/**
	 * Set the LEDs on the controller based on the match state.
	 * Green LED indicates green artifact needed, purple LED indicates purple
	 * artifact needed.
	 */
	private void setControllerLEDs() {
		final Artifact.Color neededArtifact = MatchState.nextArtifactNeeded();
		if (neededArtifact == Artifact.Color.GREEN) {
			subController.setLedColor(0, 255, 0, 100);
		} else if (neededArtifact == Artifact.Color.PURPLE) {
			subController.setLedColor(255, 0, 255, 100);
		} else {
			subController.setLedColor(0, 0, 0, 0);
		}
	}
	
	private void setupLogging() {
		// Set up lazy evaluation for frequently-accessed but expensive operations
		// These are only evaluated when telemetry is actually transmitted
		logging.addDataLazy("Heading°", () -> Math.toDegrees(mechanisms.bentDrivetrain.follower.getHeading()));
		logging.addDataLazy("X", "%.2f", () -> mechanisms.bentDrivetrain.follower.getPose().getX());
		logging.addDataLazy("Y", "%.2f", () -> mechanisms.bentDrivetrain.follower.getPose().getY());
		
		// Transfer telemetry
		mechanisms.ifValid(mechanisms.get(VerticalWheelTransfer.class), transfer -> {
			logging.addDataLazy("Transfer Slots", () -> {
				Artifact[] slots = transfer.getArtifactSnapshot();
				transferSlotsDisplayBuilder.setLength(0);
				for (int i = 0; i < slots.length; i++) {
					transferSlotsDisplayBuilder.append(i)
							.append(' ')
							.append(slots[i])
							.append(',');
				}
				return transferSlotsDisplayBuilder.toString();
			});
		});
		
		mechanisms.ifValid(mechanisms.get(PairedLauncher.class), launcher -> {
			logging.addDataLazy("Launch Solution Absolute Pitch°", () -> {
				TrajectoryEngine.AimingSolution solution = mechanisms.trajectoryEngine
						.getAimingOffsets(MatchState.getAllianceColor(), launcher.getPitch());
				return solution.hasTarget ? String.format("%.2f", solution.verticalOffsetDegrees) : "N/A";
			});
			
			logging.addDataLazy("Launch Solution Required RPM", () -> {
				TrajectoryEngine.AimingSolution solution = mechanisms.trajectoryEngine
						.getAimingOffsets(MatchState.getAllianceColor(), launcher.getPitch());
				return solution.hasTarget ? String.format("%.0f", solution.rpm) : "N/A";
			});
			
			logging.addDataLazy("Belt RPM", launcher::getRPM);
		});
		
		// Launcher mechanism status
		mechanisms.ifValid(mechanisms.get(PairedLauncher.class), launcher -> {
			logging.addDataLazy("Current Pitch°", () -> String.format("%.2f", launcher.getPitch()));
		});
		
		logging.addDataLazy("pathing", () -> {
			if (mechanisms.bentDrivetrain.getState() == BentDrivetrain.State.PATHING) {
				return mechanisms.bentDrivetrain.follower.getCurrentPath().endPose().toString();
			} else {
				return "direct drive";
			}
		});
		
		logging.addDataLazy("pathing from", () -> {
			if (mechanisms.bentDrivetrain.getState() == BentDrivetrain.State.PATHING) {
				return mechanisms.bentDrivetrain.follower.getPose().toString();
			} else {
				return "direct drive";
			}
		});
		
		logging.addDataLazy("Following", () -> mechanisms.bentDrivetrain.follower.isBusy());
		
	}
	
	private void setControllerRumble() {
		final BentDrivetrain bentDrivetrain = this.bentDrivetrain;
		if (bentDrivetrain != null) {
			final Pose currentPose = bentDrivetrain.follower.getPose();
			if (currentPose.distanceFrom(bentDrivetrain.getPositionPose(BentDrivetrain.Position.CLOSE_SHOOT)) < 1 ||
					currentPose.distanceFrom(bentDrivetrain.getPositionPose(BentDrivetrain.Position.FAR_SHOOT)) < 1) {
				subController.rumble(100);
			}
		}
	}
}
