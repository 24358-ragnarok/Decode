package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.RESET_POSE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.software.Controller;
import org.firstinspires.ftc.teamcode.software.Drivetrain;

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
		mechanisms.drivetrain.follower.startTeleopDrive();
	}
	
	/**
	 * Runs repeatedly after "start" is pressed on the Driver Station, during the
	 * actual game.
	 */
	@Override
	public final void loop() {
		mechanisms.update();
		
		processControllerInputs();
		setControllerLEDs();
		
		// Log everything that happened
		logging.drawDebug(mechanisms.drivetrain.follower);
		logging.addNumber("Heading°", Math.toDegrees(mechanisms.drivetrain.follower.getHeading()));
		logging.addNumber("X", mechanisms.drivetrain.follower.getPose().getX());
		logging.addNumber("Y", mechanisms.drivetrain.follower.getPose().getY());
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
		
		ifMechanismValid(mechanisms.drivetrain, dt -> dt.mecanumDrive(drive, strafe, rotate));
		
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
		if (subController.wasJustPressed(Controller.Action.AIM) &&
				mechanisms.alignmentEngine.isInLaunchZone(mechanisms.drivetrain.follower.getPose())) {
			mechanisms.alignmentEngine.run();
			ifMechanismValid(mechanisms.get(Launcher.class), Launcher::ready);
		} else {
			ifMechanismValid(mechanisms.get(Launcher.class), Launcher::stop);
		}
		
		if (subController.wasJustPressed(Controller.Action.LAUNCH)) {
			if (mechanisms.alignmentEngine.isInLaunchZone(mechanisms.drivetrain.getPose())) {
				ifMechanismValid(mechanisms.get(Launcher.class), Launcher::launch);
			}
		}
		
		// Intake & Spindex
		if (subController.getProcessedValue(Controller.Action.INTAKE) > 0) {
			ifMechanismValid(mechanisms.get(Spindex.class), Spindex::prepareForIntake);
			ifMechanismValid(mechanisms.get(Intake.class), Intake::in);
		} else {
			ifMechanismValid(mechanisms.get(Intake.class), Intake::stop);
		}
		
		if (mechanisms.drivetrain.getState() == Drivetrain.State.PATHING) {
			logging.addData("headed to", mechanisms.drivetrain.follower.getCurrentPath().endPose());
			logging.addData("from", mechanisms.drivetrain.follower.getPose());
		}
		if (mechanisms.drivetrain.follower.isBusy()) {
			logging.addLine("FOLLOWER IS BUSY");
		}
		Pose targetPose = (matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE)
				? Settings.Field.BLUE_GOAL_POSE
				: Settings.Field.RED_GOAL_POSE;
		logging.addNumber("angle to goal",
				Math.toDegrees(mechanisms.alignmentEngine.angleToTarget(mechanisms.drivetrain.getPose(),
						targetPose)));
		
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
