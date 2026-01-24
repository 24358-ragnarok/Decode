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
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.Lever;
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
	private Drivetrain drivetrain;
	private VerticalWheelTransfer transfer;
	private FlexVectorIntake intake;
	private PairedLauncher launcher;
	private BallSwap swap;
	private Lever lever;
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
		drivetrain = mechanisms.drivetrain;
		transfer = mechanisms.get(VerticalWheelTransfer.class);
		intake = mechanisms.get(FlexVectorIntake.class);
		launcher = mechanisms.get(PairedLauncher.class);
		swap = mechanisms.get(BallSwap.class);
		lever = mechanisms.get(Lever.class);
		
		mainController = new Controller(gamepad1, mechanisms.drivetrain.follower,
				PanelsGamepad.INSTANCE.getFirstManager());
		subController = new Controller(gamepad2, mechanisms.drivetrain.follower,
				PanelsGamepad.INSTANCE.getSecondManager());
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		mechanisms.drivetrain.follower.setStartingPose(MatchState.getTeleOpStartingPose());
		mechanisms.drivetrain.switchToManual();
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
		logging.drawDebug(mechanisms.drivetrain.follower);
		logging.update();
	}
	
	/**
	 * Runs when "start" is pressed on the Driver Station.
	 */
	@Override
	public final void start() {
		// Initialize mechanisms and start teleop drive
		mechanisms.ifValid(mechanisms, MechanismManager::start);
		mechanisms.drivetrain.follower.startTeleopDrive(); // or pass in true to enable braking
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
		log.drawDebug(mech.drivetrain.follower);
		log.update();
	}
	
	/**
	 * Runs when "stop" is pressed on the Driver Station.
	 * Cleanup and shutdown should occur instantaneously and be non-blocking.
	 */
	@Override
	public final void stop() {
		// Store the actual robot pose for future reference or debugging
		if (mechanisms != null && mechanisms.drivetrain.follower != null) {
			MatchState.setStoredPose(mechanisms.drivetrain.follower.getPose());
			mechanisms.stop();
		}
	}
	
	/**
	 * Process controller inputs for both main and sub controllers.
	 * Handles drivetrain movement, launcher controls, intake operations, and
	 * telemetry updates.
	 */
	private void processControllerInputs() {
		// Refresh gamepad state FIRST (required for edge detection to work)
		mainController.update();
		subController.update();
		
		// Hot-loop locals
		final Drivetrain drivetrain = this.drivetrain;
		final VerticalWheelTransfer transfer = this.transfer;
		final FlexVectorIntake intake = this.intake;
		final PairedLauncher launcher = this.launcher;
		final BallSwap swap = this.swap;
		final Lever lever = this.lever;
		
		// Drivetrain
		double d = mainController.getProcessedDrive();
		double s = mainController.getProcessedStrafe();
		double r = mainController.getProcessedRotation();
		
		if (drivetrain != null) {
			if (mainController.wasJustPressed(Controller.Action.TOGGLE_CENTRICITY)) {
				drivetrain.toggleCentricity();
			}
			
			drivetrain.manual(d, s, r);
			
			if (mainController.getProcessedValue(Controller.Action.SET_FOLLOWER) > 0.5) {
				if (mainController.wasJustPressed(Controller.Action.GOTO_GATE)) {
					drivetrain.follower.setPose(
							(MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE)
									? Settings.Positions.Reset.HUMAN_PLAYER_ZONE
									: Settings.Field.mirrorPose(Settings.Positions.Reset.HUMAN_PLAYER_ZONE));
				}
				if (mainController.wasJustPressed(Controller.Action.GOTO_CLOSE_SHOOT)) {
					drivetrain.follower.setPose(
							(MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE)
									? Settings.Positions.Reset.CLOSE_ZONE
									: Settings.Field.mirrorPose(Settings.Positions.Reset.CLOSE_ZONE));
				}
				if (mainController.wasJustPressed(Controller.Action.GOTO_FAR_SHOOT)) {
					drivetrain.follower.setPose(
							(MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE)
									? Settings.Positions.Reset.FAR_ZONE
									: Settings.Field.mirrorPose(Settings.Positions.Reset.FAR_ZONE));
				}
				if (mainController.wasJustPressed(Controller.Action.GOTO_PARK)) {
					if (mechanisms.limelightManager != null) {
						double currentHeading = drivetrain.follower.getHeading();
						Pose estimatedPose = mechanisms.limelightManager.estimateRobotPose(currentHeading);
						if (estimatedPose != null) {
							drivetrain.follower.setPose(estimatedPose);
						}
					}
				}
			} else {
				
				final double startValue = mainController.getProcessedValue(Controller.Control.OPTIONS);
				for (Controller.Action action : Settings.Controls.gotoActions) {
					if (mainController.wasJustPressed(action) && startValue <= 0.0) {
						drivetrain.goTo(action);
						// Stop intake when going to park
						if (drivetrain.actionToPosition(action) == Drivetrain.Position.PARK && transfer != null) {
							transfer.freeze();
						}
					} else if (mainController.wasJustReleased(action)) {
						drivetrain.switchToManual();
					}
				}
				if (mainController.wasJustPressed(Controller.Control.LEFT_TRIGGER)) {
					drivetrain.goTo(drivetrain.follower.getPose().withHeading(drivetrain.follower.getHeading() + mechanisms.limelightManager.estimateHeadingToGoal()));
				} else if (mainController.wasJustReleased(Controller.Control.LEFT_TRIGGER)) {
					drivetrain.switchToManual();
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
				launcher.openDynamic();
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
				swap.moveToTransfer();
			}
			if (subController.wasJustPressed(Controller.Control.LEFT_BUMPER)) {
				swap.moveToHold();
			}
		}
		
		logging.addData("1", mainController.getProcessedValue(Controller.Action.PARK_EXTEND_1));
		logging.addData("2", mainController.getProcessedValue(Controller.Action.PARK_EXTEND_2));
		if (lever != null) {
			if (mainController.getProcessedValue(Controller.Action.PARK_EXTEND_1) > 0.5 &&
					mainController.getProcessedValue(Controller.Action.PARK_EXTEND_2) > 0.5) {
				lever.extend();
			} else if (lever.isExtended() && mainController.wasJustPressed(Controller.Action.PARK_EXTEND_1)) {
				lever.retract();
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
		logging.addDataLazy("heading°", () -> Math.toDegrees(mechanisms.drivetrain.follower.getHeading()));
		logging.addDataLazy("x", "%.2f", () -> mechanisms.drivetrain.follower.getPose().getX());
		logging.addDataLazy("y", "%.2f", () -> mechanisms.drivetrain.follower.getPose().getY());
		
		mechanisms.ifValid(mechanisms.get(PairedLauncher.class), launcher -> {
			logging.addDataLazy("Launch Solution Absolute Pitch°", () -> {
				TrajectoryEngine.AimingSolution solution = mechanisms.trajectoryEngine
						.getAimingOffsets(MatchState.getAllianceColor(), launcher.getPitch());
				return solution.hasTarget ? String.format("%.2f", solution.pitch) : "N/A";
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
			if (mechanisms.drivetrain.getState() == Drivetrain.State.PATHING) {
				return mechanisms.drivetrain.follower.getCurrentPath().endPose().toString();
			} else {
				return "direct drive";
			}
		});
		
		logging.addDataLazy("pathing from", () -> {
			if (mechanisms.drivetrain.getState() == Drivetrain.State.PATHING) {
				return mechanisms.drivetrain.follower.getPose().toString();
			} else {
				return "direct drive";
			}
		});
		
		logging.addDataLazy("Following", () -> mechanisms.drivetrain.follower.isBusy());
		
	}
	
	private void setControllerRumble() {
		final Drivetrain drivetrain = this.drivetrain;
		if (drivetrain != null) {
			final Pose currentPose = drivetrain.follower.getPose();
			if (currentPose.distanceFrom(drivetrain.getPositionPose(Drivetrain.Position.CLOSE_SHOOT)) < 1 ||
					currentPose.distanceFrom(drivetrain.getPositionPose(Drivetrain.Position.FAR_SHOOT)) < 1) {
				subController.rumble(100);
			}
		}
	}
}
