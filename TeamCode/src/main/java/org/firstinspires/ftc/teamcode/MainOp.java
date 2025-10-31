package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.RESET_POSE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.hardware.FlywheelIntake;
import org.firstinspires.ftc.teamcode.hardware.HorizontalLauncher;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;
import org.firstinspires.ftc.teamcode.software.Controller;
import org.firstinspires.ftc.teamcode.software.Drivetrain;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

import java.util.function.Consumer;

/**
 * This is our main TeleOp class for the driver-controlled period, which occurs
 * after Auto.
 * Handles controller profile selection and robot operation during matches.
 */
@TeleOp(name = "MainOp", group = ".Competition Modes")
public class MainOp extends OpMode {
	public MatchSettings matchSettings;
	private UnifiedLogging logging;
	private MechanismManager mechanisms;
	private Controller mainController;
	private Controller subController;
	
	/**
	 * This allows us to run commands only if the related mechanism works.
	 * For example I could run "if launcher exists, shoot it" using this.
	 */
	private static <T> void ifMechanismValid(T obj, Consumer<T> action) {
		if (obj != null) {
			action.accept(obj);
		}
	}
	
	/**
	 * Runs when "init" is pressed on the Driver Station.
	 */
	@Override
	public final void init() {
		// Pull the stored match state and settings from when they were set during auto
		matchSettings = new MatchSettings(blackboard);
		
		// Initialize robot systems
		mechanisms = new MechanismManager(hardwareMap, matchSettings);
		mainController = new Controller(gamepad1, mechanisms.drivetrain.follower, matchSettings);
		subController = new Controller(gamepad2, mechanisms.drivetrain.follower, matchSettings);
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		
		// Enable retained mode for efficient telemetry updates
		logging.enableRetainedMode();
		
		// Set up lazy evaluation for frequently-accessed but expensive operations
		// These are only evaluated when telemetry is actually transmitted
		logging.addDataLazy("Heading°", () -> Math.toDegrees(mechanisms.drivetrain.follower.getHeading()));
		logging.addDataLazy("X", "%.2f", () -> mechanisms.drivetrain.follower.getPose().getX());
		logging.addDataLazy("Y", "%.2f", () -> mechanisms.drivetrain.follower.getPose().getY());
		
		mechanisms.drivetrain.follower.setStartingPose(matchSettings.getTeleOpStartingPose());
		mechanisms.drivetrain.switchToManual();
	}
	
	/**
	 * Runs after "init" and before "start" repeatedly.
	 */
	@Override
	public final void init_loop() {
		logging.drawDebug(mechanisms.drivetrain.follower);
	}
	
	/**
	 * Runs when "start" is pressed on the Driver Station.
	 */
	@Override
	public final void start() {
		// Initialize mechanisms and start teleop drive
		ifMechanismValid(mechanisms, MechanismManager::init);
		mechanisms.drivetrain.follower.startTeleopDrive(); // or pass in true to enable braking
	}
	
	/**
	 * Runs repeatedly after "start" is pressed on the Driver Station, during the
	 * actual game.
	 */
	@Override
	public final void loop() {
		mechanisms.update();
		
		// Clear dynamic (non-retained) telemetry from previous loop
		logging.clearDynamic();
		
		processControllerInputs();
		setControllerLEDs();
		
		// Draw debug visualization (retained items like Heading, X, Y are auto-updated
		// via Func)
		logging.drawDebug(mechanisms.drivetrain.follower);
		logging.update();
	}
	
	/**
	 * Runs when "stop" is pressed on the Driver Station.
	 * Cleanup and shutdown should occur instantaneously and be non-blocking.
	 */
	@Override
	public final void stop() {
		mechanisms.stop();
		blackboard.clear(); // do not save match settings in between matches
	}
	
	/**
	 * Process controller inputs
	 */
	private void processControllerInputs() {
		// Drivetrain
		double drive = mainController.getProcessedDrive();
		double strafe = mainController.getProcessedStrafe();
		double rotate = mainController.getProcessedRotation();
		
		if (mainController.wasJustPressed(Controller.Action.TOGGLE_CENTRICITY)) {
			ifMechanismValid(mechanisms.drivetrain, Drivetrain::toggleCentricity);
		}
		
		if (mainController.wasJustPressed(Controller.Action.RESET_FOLLOWER)) {
			ifMechanismValid(mechanisms.drivetrain, dt -> dt.follower.setPose(RESET_POSE));
		}
		
		// automatically switch the perspective for field-centric
		// driving based on the side we start on
		double perspectiveRotation = matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE
				? Math.toRadians(0)
				: Math.toRadians(180);
		
		ifMechanismValid(mechanisms.drivetrain, dt -> dt.manual(drive, strafe, rotate, perspectiveRotation));
		
		// Go-to actions
		Controller.Action[] gotoActions = {
				Controller.Action.GOTO_CLOSE_SHOOT,
				Controller.Action.GOTO_FAR_SHOOT,
				Controller.Action.GOTO_HUMAN_PLAYER,
				Controller.Action.GOTO_GATE
		};
		for (Controller.Action action : gotoActions) {
			if (mainController.wasJustPressed(action)
					&& mainController.getProcessedValue(Controller.Control.START) <= 0.0) {
				ifMechanismValid(mechanisms.drivetrain,
						dt -> dt.goTo(Drivetrain.Position.valueOf(action.name().substring("GOTO_".length()))));
				break;
			}
			if (mainController.getProcessedValue(action) > 0) {
				logging.addData("goto", action);
			}
		}
		
		if (mainController.wasJustPressed(Controller.Action.CANCEL_ASSISTED_DRIVING)) {
			ifMechanismValid(mechanisms.drivetrain, Drivetrain::switchToManual);
		}
		
		// Alignment & Launcher
		if (subController.getProcessedValue(Controller.Action.AIM) > 0.1) {
			mechanisms.alignmentEngine.run();
			ifMechanismValid(mechanisms.get(HorizontalLauncher.class), HorizontalLauncher::ready);
		} else {
			ifMechanismValid(mechanisms.get(HorizontalLauncher.class), HorizontalLauncher::stop);
			if (subController.wasJustReleased(Controller.Action.AIM)) {
				mechanisms.drivetrain.switchToManual();
			}
		}
		
		if (subController.wasJustPressed(Controller.Action.LAUNCH)) {
			if (mechanisms.alignmentEngine.isInLaunchZone(mechanisms.drivetrain.getPose())) {
				// Fire the transfer when launcher is ready
				HorizontalLauncher launcher = mechanisms.get(HorizontalLauncher.class);
				boolean launcherReady = launcher == null || launcher.okayToLaunch();
				
				if (launcherReady) {
					ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), SingleWheelTransfer::fire);
				}
			}
		}
		
		// TODO REMOVE debug experimental subsystem commands
		if (subController.wasJustPressed(Controller.Control.DPAD_UP)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), SingleWheelTransfer::advance);
		}
		if (subController.wasJustPressed(Controller.Control.DPAD_RIGHT)) {
			ifMechanismValid(mechanisms.get(FlywheelIntake.class), FlywheelIntake::in);
		}
		
		ifMechanismValid(mechanisms.get(FlywheelIntake.class),
				fwi -> {
					logging.addLine("Intake valid");
				});
		
		// Debug telemetry for aiming system
		logging.addLine("=== AIM DEBUG ===");
		logging.addData("Aim Button", subController.getProcessedValue(Controller.Action.AIM) > 0.1);
		logging.addData("In Launch Zone", mechanisms.alignmentEngine.isInLaunchZone(mechanisms.drivetrain.getPose()));
		logging.addData("Alliance", matchSettings.getAllianceColor());
		logging.addData("Target Tag ID",
				matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE ? 20 : 24);
		
		// Get aiming solution for debugging
		ifMechanismValid(mechanisms.get(HorizontalLauncher.class),
				launcher -> {
					TrajectoryEngine.AimingSolution solution = mechanisms.trajectoryEngine
							.getAimingOffsets(matchSettings.getAllianceColor(), launcher.getPitch());
					if (solution.hasTarget) {
						logging.addData("Yaw Offset°", "%.2f", solution.horizontalOffsetDegrees);
						logging.addData("Pitch Offset°", "%.2f", solution.verticalOffsetDegrees);
						logging.addData("Required RPM", "%.0f", solution.rpm);
					}
				});
		
		// Launcher mechanism status
		ifMechanismValid(mechanisms.get(HorizontalLauncher.class), hl -> {
			logging.addData("Launcher Ready", hl.okayToLaunch());
			logging.addData("Current Pitch°", "%.2f", hl.getPitch());
			logging.addData("Current Yaw°", "%.2f", hl.getYaw());
		});
		
		// Intake & Transfer
		if (subController.getProcessedValue(Controller.Action.INTAKE) > 0) {
			ifMechanismValid(mechanisms.get(FlywheelIntake.class), FlywheelIntake::in);
		} else {
			ifMechanismValid(mechanisms.get(FlywheelIntake.class), FlywheelIntake::stop);
		}
		
		// Manual transfer controls
		if (subController.wasJustPressed(Controller.Action.RELEASE_EXTRAS)) {
			// Advance next ball to kicker position
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), SingleWheelTransfer::advance);
		}
		
		// Color-specific ball advancement
		if (subController.wasJustPressed(Controller.Action.RELEASE_GREEN)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), transfer -> {
				int greenIndex = transfer.indexOf(MatchSettings.ArtifactColor.GREEN);
				if (greenIndex >= 0) {
					transfer.moveSlotToKicker(greenIndex);
				}
			});
		}
		
		if (subController.wasJustPressed(Controller.Action.RELEASE_PURPLE)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), transfer -> {
				int purpleIndex = transfer.indexOf(MatchSettings.ArtifactColor.PURPLE);
				if (purpleIndex >= 0) {
					transfer.moveSlotToKicker(purpleIndex);
				}
			});
		}
		
		// Transfer telemetry
		ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), transfer -> {
			logging.addLine("=== TRANSFER STATUS ===");
			logging.addData("Next at Exit", transfer.getNextBallToKicker());
			logging.addData("Full", transfer.isFull());
			logging.addData("Empty", transfer.isEmpty());
			logging.addData("Transfer Wheel", transfer.isTransferWheelRunning() ? "RUNNING" : "STOPPED");
			logging.addData("Entrance Wheel", transfer.isEntranceWheelOpen() ? "OPEN" : "CLOSED");
			logging.addData("Exit Wheel", transfer.isExitWheelFiring() ? "FIRING" : "CLOSED");
			
			// Show slot contents
			MatchSettings.ArtifactColor[] slots = transfer.getSlotsSnapshot();
			StringBuilder slotsDisplay = new StringBuilder();
			for (int i = 0; i < slots.length; i++) {
				slotsDisplay.append(i).append(":").append(slots[i]).append(" ");
			}
			logging.addData("Slots", slotsDisplay.toString());
		});
		
		// Dynamic telemetry - only shown when conditions are met
		if (mechanisms.drivetrain.getState() == Drivetrain.State.PATHING) {
			logging.addData("headed to", mechanisms.drivetrain.follower.getCurrentPath().endPose());
			logging.addData("from", mechanisms.drivetrain.follower.getPose());
		}
		if (mechanisms.drivetrain.follower.isBusy()) {
			logging.addLine("FOLLOWER IS BUSY");
		}
		
		// Use lazy evaluation for expensive angle calculation - only computed when
		// transmitted
		Pose targetPose = (matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE)
				? Settings.Field.BLUE_GOAL_POSE
				: Settings.Field.RED_GOAL_POSE;
		logging.addDataLazy("angle to goal", "%.2f",
				() -> Math.toDegrees(mechanisms.alignmentEngine.angleToTarget(
						mechanisms.drivetrain.getPose(), targetPose)));
		
		// Classifier controls
		if (subController.getProcessedValue(Controller.Action.EMPTY_CLASSIFIER_STATE) > 0)
			matchSettings.emptyClassifier();
		
		if (subController.getProcessedValue(Controller.Action.INCREMENT_CLASSIFIER_STATE) > 0)
			matchSettings.incrementClassifier();
		
		mainController.saveLastState();
		subController.saveLastState();
	}
	
	/**
	 * Set the LEDs on the controller based on the match state.
	 */
	private void setControllerLEDs() {
		if (matchSettings.nextArtifactNeeded() == MatchSettings.ArtifactColor.GREEN) {
			subController.setLedColor(0, 255, 0, 100);
		} else if (matchSettings.nextArtifactNeeded() == MatchSettings.ArtifactColor.PURPLE) {
			subController.setLedColor(255, 0, 255, 100);
		} else {
			subController.setLedColor(0, 0, 0, 0);
		}
	}
}
