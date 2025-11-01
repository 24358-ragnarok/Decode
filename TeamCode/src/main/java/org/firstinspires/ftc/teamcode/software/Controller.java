package org.firstinspires.ftc.teamcode.software;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;

import java.util.HashMap;

/**
 * @noinspection CyclicClassDependency, DataFlowIssue
 * The Controller class wraps around the FTC-provided
 * {@link Gamepad}.
 * It provides versatility and does complex calculations to allow
 * us to have a cleanly controlled
 * TeleOp.
 */
public class Controller extends Gamepad {
	private final Gamepad gamepad;
	private final HashMap<Control, Double> previousControlState;
	private final Follower follower;
	
	private final MatchSettings matchSettings;
	
	public Controller(Gamepad gamepad, Follower follower, MatchSettings matchSettings) {
		this.gamepad = gamepad;
		this.follower = follower;
		this.matchSettings = matchSettings;
		this.previousControlState = new HashMap<>();
		// Populate the initial state in the constructor
		saveLastState();
	}
	
	/**
	 * Stores the previous state of the controller.
	 * This is useful for figuring out if we just started pressing a button or if it
	 * has been held.
	 */
	public final void saveLastState() {
		for (Control control : Control.values()) {
			previousControlState.put(control, getRawValue(control));
		}
	}
	
	/**
	 * Checks if an control was just pressed.
	 *
	 * @param control The control to check
	 * @return True if that control was pressed this frame and not last frame, False
	 * otherwise
	 */
	public final boolean wasJustPressed(Control control) {
		return getRawValue(control) != 0.0 && previousControlState.getOrDefault(control, 0.0) == 0;
	}
	
	/**
	 * Checks if an action was just pressed.
	 *
	 * @param action The action to check
	 * @return True if that action was pressed this frame and not last frame, False
	 * otherwise
	 */
	public final boolean wasJustPressed(Action action) {
		return wasJustPressed(getControlForAction(action));
	}
	
	/**
	 * Checks if an control was just released.
	 *
	 * @param control The control to check
	 * @return True if that control was pressed last frame and not this frame, False
	 * otherwise
	 */
	public final boolean wasJustReleased(Control control) {
		return getRawValue(control) == 0.0 && previousControlState.getOrDefault(control, 0.0) != 0;
	}
	
	/**
	 * Checks if an action was just released.
	 *
	 * @param action The action to check
	 * @return True if that action was pressed last frame and not this frame, False
	 * otherwise
	 */
	public final boolean wasJustReleased(Action action) {
		return wasJustReleased(getControlForAction(action));
	}
	
	/**
	 * Maps each action to a control. This allows us to ask if the action "Spin" was
	 * pressed instead
	 * of hard-coding what control makes it spin.
	 *
	 * @param action The action to get the control for
	 * @return What control on the controller corresponds to a given Action
	 */
	public final Control getControlForAction(Action action) {
		return Settings.Controls.actionControlMap.getOrDefault(action, Control.UNKNOWN);
	}
	
	/**
	 * Applies normalization to raw values. For example, the left stick Y is
	 * automatically inverted.
	 * This should be used to normalize controls, not to modify them.
	 * In this example, the left stick Y is inverted so that it follows the
	 * normalized Y-axis of the robot.
	 * This does not cause a change in function of the how the value operates, which
	 * should be done in postprocessing.
	 *
	 * @param control The control to get the value for
	 * @return The processed value for that control
	 */
	public final double getProcessedValue(Control control) {
		// add value modifiers here
		double val = getRawValue(control);
		
		if (control == Control.LEFT_STICK_Y || control == Control.LEFT_STICK_X) {
			val = -val;
		}
		// add more here
		
		return val;
	}
	
	public final double getProcessedValue(Action action) {
		return getProcessedValue(getControlForAction(action));
	}
	
	/**
	 * Makes the Dpad of the controller move the robot absolutely on the field by
	 * translating the
	 * inputs to the robot's coordinate system.
	 *
	 * @return The processed robot-centric movement vector
	 */
	public Vector getRobotCentricDpad() {
		// Calculate the field-centric vector components from d-pad inputs.
		// North/East are positive.
		double fieldY = getProcessedValue(Action.ABS_NORTH) - getProcessedValue(Action.ABS_SOUTH);
		double fieldX = getProcessedValue(Action.ABS_EAST) - getProcessedValue(Action.ABS_WEST);
		
		Vector dpadVector = new Vector(fieldX, fieldY);
		
		// To convert a field-centric vector to a robot-centric one, rotate it opposite
		// the robot's current heading.
		dpadVector.rotateVector(-follower.getHeading());
		
		return dpadVector;
	}
	
	/**
	 * Processes inputs related to the robot's forward movement.
	 *
	 * @return The processed forward movement value (positive = forward, negative =
	 * backward)
	 */
	public final double getProcessedDrive() {
		Vector robotVec = getRobotCentricDpad();
		double drive = getProcessedValue(Action.MOVE_Y) + robotVec.getYComponent();
		return Math.max(-1, Math.min(1, drive));
	}
	
	/**
	 * Processes inputs related to the robot's strafe movement.
	 *
	 * @return The processed strafe movement value (negative = left, positive =
	 * right)
	 */
	public final double getProcessedStrafe() {
		Vector robotVec = getRobotCentricDpad();
		double strafe = getProcessedValue(Action.MOVE_X) + robotVec.getXComponent();
		return Math.max(-1, Math.min(1, strafe));
	}
	
	/**
	 * Processes inputs related to the robot's rotation.
	 *
	 * @return The processed rotation value (positive = clockwise, negative =
	 * counterclockwise)
	 */
	public final double getProcessedRotation() {
		double rotationValue = getProcessedValue(Action.ROTATE_AXIS) + getProcessedValue(Action.ROTATE_RIGHT)
				- getProcessedValue(Action.ROTATE_LEFT);
		return Math.max(-1, Math.min(1, rotationValue));
	}
	
	/**
	 * @param control The control to get the value for
	 * @return The value of the control
	 * @noinspection OverlyComplexMethod, OverlyLongMethod
	 * Never edit this.
	 * Returns the raw value of a control by interfacing with the FTC
	 * {@link Gamepad}.
	 */
	private double getRawValue(Control control) {
		switch (control) {
			case LEFT_TRIGGER:
				return gamepad.left_trigger;
			case RIGHT_TRIGGER:
				return gamepad.right_trigger;
			case LEFT_STICK_X:
				return gamepad.left_stick_x;
			case LEFT_STICK_Y:
				return gamepad.left_stick_y;
			case RIGHT_STICK_X:
				return gamepad.right_stick_x;
			case RIGHT_STICK_Y:
				return gamepad.right_stick_y;
			case BACK:
				return gamepad.back ? 1 : 0;
			case START:
				return gamepad.start ? 1 : 0;
			case CIRCLE:
				return gamepad.circle ? 1 : 0;
			case CROSS:
				return gamepad.cross ? 1 : 0;
			case SQUARE:
				return gamepad.square ? 1 : 0;
			case TRIANGLE:
				return gamepad.triangle ? 1 : 0;
			case LEFT_BUMPER:
				return gamepad.left_bumper ? 1 : 0;
			case RIGHT_BUMPER:
				return gamepad.right_bumper ? 1 : 0;
			case DPAD_UP:
				return gamepad.dpad_up ? 1 : 0;
			case DPAD_DOWN:
				return gamepad.dpad_down ? 1 : 0;
			case DPAD_LEFT:
				return gamepad.dpad_left ? 1 : 0;
			case DPAD_RIGHT:
				return gamepad.dpad_right ? 1 : 0;
			case LEFT_STICK_BUTTON:
				return gamepad.left_stick_button ? 1 : 0;
			case RIGHT_STICK_BUTTON:
				return gamepad.right_stick_button ? 1 : 0;
			case GUIDE:
				return gamepad.guide ? 1 : 0;
			case OPTIONS:
				return gamepad.options ? 1 : 0;
			case TOUCHPAD:
				return gamepad.touchpad ? 1 : 0;
			case TOUCHPAD_X:
				return gamepad.touchpad_finger_1_x;
			case TOUCHPAD_Y:
				return gamepad.touchpad_finger_1_y;
			default:
				return 0;
		}
	}
	
	/**
	 * Actions are representations of what the driver WANTS the robot to do when
	 * they press a button.
	 * For example, if we've mapped Cross to Shoot, the driver wants the robot to
	 * Action.SHOOT when cross is pressed.
	 * This is useful to decouple because we can easily remap controls while keeping
	 * the code clean.
	 */
	public enum Action {
		MOVE_Y,
		MOVE_X,
		ROTATE_LEFT,
		ROTATE_RIGHT,
		ROTATE_AXIS,
		ABS_NORTH,
		ABS_EAST,
		ABS_WEST,
		ABS_SOUTH,
		TOGGLE_CENTRICITY,
		RESET_FOLLOWER,
		GOTO_CLOSE_SHOOT,
		GOTO_FAR_SHOOT,
		GOTO_HUMAN_PLAYER,
		GOTO_GATE,
		CANCEL_ASSISTED_DRIVING,
		INTAKE_IN,
		INTAKE_OUT,
		OVERRIDE_BALL_DETECTION,
		OVERRIDE_ADVANCE,
		OVERRIDE_LAUNCH,
		OVERRIDE_SPINUP,
		AIM,
		LAUNCH,
		LAUNCHER_STEEPNESS_AXIS,
		LAUNCHER_ROTATION_AXIS,
		PARK_EXTEND,
		INCREMENT_CLASSIFIER_STATE,
		EMPTY_CLASSIFIER_STATE,
		UNSET,
	}
	
	/**
	 * Controls are the counterparts to {@link Action}s. They represent the actual
	 * buttons that will be pressed
	 * to make an action happen. These are mapped in settings, and should be
	 * accessed through Actions
	 * instead of directly.
	 */
	public enum Control {
		TRIANGLE, CIRCLE, CROSS, SQUARE,
		DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
		LEFT_BUMPER, RIGHT_BUMPER,
		START, BACK, GUIDE, OPTIONS,
		LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
		TOUCHPAD, TOUCHPAD_X, TOUCHPAD_Y,
		
		LEFT_TRIGGER, RIGHT_TRIGGER,
		LEFT_STICK_X, LEFT_STICK_Y,
		RIGHT_STICK_X, RIGHT_STICK_Y,
		
		UNKNOWN
	}
}