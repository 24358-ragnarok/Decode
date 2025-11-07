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
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
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
	public static <T> void ifMechanismValid(T obj, Consumer<T> action) {
		if (obj != null) {
			action.accept(obj);
		}
	}

	/**
	 * Runs when "init" is pressed on the Driver Station.
	 * Initializes all robot systems, controllers, and telemetry for TeleOp
	 * operation.
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

		// Set up lazy evaluation for frequently-accessed but expensive operations
		// These are only evaluated when telemetry is actually transmitted
		logging.addDataLazy("Heading°", () -> Math.toDegrees(mechanisms.drivetrain.follower.getHeading()));
		logging.addDataLazy("X", "%.2f", () -> mechanisms.drivetrain.follower.getPose().getX());
		logging.addDataLazy("Y", "%.2f", () -> mechanisms.drivetrain.follower.getPose().getY());

		logging.addData("Alliance", matchSettings.getAllianceColor());
		
		// Show whether we're using stored pose or fallback
		Pose storedPose = matchSettings.getStoredPose();
		if (storedPose != null) {
			logging.addData("Starting Pose Source", "STORED FROM PREVIOUS");
			logging.addData("Stored Pose", storedPose);
		} else {
			logging.addData("Starting Pose Source", "FALLBACK/PREDEFINED");
		}
		
		for (Mechanism m : mechanisms.mechanismArray) {
			logging.addData(m.getClass().toString(), "✅");
		}
		logging.update();

		mechanisms.drivetrain.follower.setStartingPose(matchSettings.getTeleOpStartingPose());
		mechanisms.drivetrain.switchToManual();
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
		ifMechanismValid(mechanisms, MechanismManager::start);
		mechanisms.drivetrain.follower.startTeleopDrive(); // or pass in true to enable braking
		matchSettings.clearStoredPose();
		logging.enableRetainedMode();
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
		// Store the actual robot pose for future reference or debugging
		if (mechanisms != null && mechanisms.drivetrain != null && mechanisms.drivetrain.follower != null) {
			matchSettings.setStoredPose(mechanisms.drivetrain.follower.getPose());
		}
		
		mechanisms.stop();
	}
	
	/**
	 * Process controller inputs for both main and sub controllers.
	 * Handles drivetrain movement, launcher controls, intake operations, and
	 * telemetry updates.
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
		
		// Automatically switch the perspective for field-centric
		// driving based on the alliance color we start on
		double perspectiveRotation = matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE
				? Math.toRadians(0)
				: Math.toRadians(180);
		
		ifMechanismValid(mechanisms.drivetrain, dt -> dt.manual(drive, strafe, rotate, perspectiveRotation));
		
		for (Controller.Action action : Settings.Controls.gotoActions) {
			Pose gotopose = mechanisms.drivetrain
					.getPositionPose(Drivetrain.Position.valueOf(action.name().substring("GOTO_".length())));
			if (mainController.wasJustPressed(action)
					&& mainController.getProcessedValue(Controller.Control.START) <= 0.0) {
				ifMechanismValid(mechanisms.drivetrain,
						dt -> dt.goTo(gotopose));
			} else if (mainController.wasJustReleased(action)) {
				mechanisms.drivetrain.switchToManual();
			}
		}
		
		if (mainController.wasJustPressed(Controller.Action.CANCEL_ASSISTED_DRIVING)) {
			ifMechanismValid(mechanisms.drivetrain, Drivetrain::switchToManual);
		}
		
		// Alignment & Launcher
		if (subController.getProcessedValue(Controller.Action.AIM) > 0.1) {
			ifMechanismValid(mechanisms.get(HorizontalLauncher.class), HorizontalLauncher::ready);
		} else {
			ifMechanismValid(mechanisms.get(HorizontalLauncher.class), HorizontalLauncher::stop);
		}
		
		if (subController.wasJustPressed(Controller.Action.AIM)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), SingleWheelTransfer::moveNextBallToKicker);
		}
		
		if (subController.wasJustPressed(Controller.Action.LAUNCH)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), SingleWheelTransfer::fire);
		}
		
		if (subController.wasJustPressed(Controller.Action.OVERRIDE_ADVANCE)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), SingleWheelTransfer::advance);
		}
		if (subController.wasJustPressed(Controller.Action.OVERRIDE_BALL_DETECTION)) {
			ifMechanismValid(mechanisms.get(SingleWheelTransfer.class), swt -> {
				swt.clearAllSlots();
				swt.openEntrance();
			});
		}
		
		// Get aiming solution for debugging
		ifMechanismValid(mechanisms.get(HorizontalLauncher.class),
				launcher -> {
					TrajectoryEngine.AimingSolution solution = mechanisms.trajectoryEngine
							.getAimingOffsets(matchSettings.getAllianceColor(), launcher.getPitch());
					if (solution.hasTarget) {
						logging.addData("Yaw Offset°", "%.2f", solution.horizontalOffsetDegrees);
						logging.addData("Target Pitch°", "%.2f", solution.verticalOffsetDegrees);
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
		if (subController.wasJustPressed(Controller.Action.INTAKE_IN)) {
			ifMechanismValid(mechanisms.get(FlywheelIntake.class), FlywheelIntake::in);
		}
		
		if (subController.wasJustPressed(Controller.Action.INTAKE_OUT)) {
			ifMechanismValid(mechanisms.get(FlywheelIntake.class), FlywheelIntake::out);
		}
		
		if (subController.wasJustPressed(Controller.Action.INTAKE_STOP)) {
			ifMechanismValid(mechanisms.get(FlywheelIntake.class), FlywheelIntake::stop);
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
	 * Green LED indicates green artifact needed, purple LED indicates purple
	 * artifact needed.
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
