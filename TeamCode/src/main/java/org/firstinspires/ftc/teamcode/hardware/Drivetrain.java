package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configuration.Constants;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.Controller;

import java.util.HashMap;
import java.util.Map;

/**
 * Drivetrain class refactored to use the PedroPathing V2 library.
 * It abstracts away direct motor control in favor of the Follower API for
 * both manual (tele-op) and autonomous movement.
 */
public class Drivetrain extends Mechanism {
	public final Follower follower;
	// Define field-centric poses for autonomous targets.
	// Automatically mirrored for RED alliance.
	private final Map<Position, Pose> positionPoses = new HashMap<>();
	public boolean robotCentric = true;
	private State state;

	/**
	 * Initializes the Drivetrain and the PedroPathing Follower.
	 * <p>
	 * Note: Starting pose should be set by the OpMode (MainAuto or MainOp),
	 * not in this constructor.
	 *
	 * @param hardwareMap   The robot's hardware map.
	 */
	public Drivetrain(HardwareMap hardwareMap) {
		// The Constants class now holds all hardware and tuning configurations.
		this.follower = Constants.createFollower(hardwareMap);
		// Don't set starting pose here - let each OpMode handle it

		// Initialize the poses for each predefined position
		// Use BLUE as reference, mirror for RED alliance
		boolean isBlue = MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE;

		positionPoses.put(Position.CLOSE_SHOOT,
				isBlue ? Settings.Positions.TeleOp.CLOSE_SHOOT
						: Settings.Field.mirrorPose(Settings.Positions.TeleOp.CLOSE_SHOOT));
		positionPoses.put(Position.FAR_SHOOT,
				isBlue ? Settings.Positions.TeleOp.FAR_SHOOT
						: Settings.Field.mirrorPose(Settings.Positions.TeleOp.FAR_SHOOT));
		positionPoses.put(Position.PARK,
				isBlue ? Settings.Positions.TeleOp.PARK : Settings.Field.mirrorPose(Settings.Positions.TeleOp.PARK));
		positionPoses.put(Position.GATE,
				isBlue ? Settings.Positions.TeleOp.GATE : Settings.Field.mirrorPose(Settings.Positions.TeleOp.GATE));
	}

	@Override
	public void start() {
		// No initialization required - drivetrain is initialized in constructor
	}
	
	public void toggleCentricity() {
		robotCentric = !robotCentric;
	}
	
	/**
	 * This method MUST be called in the main loop of your OpMode to keep the
	 * follower's internal state and localization updated.
	 */
	public void update() {
		follower.update();
	}
	
	@Override
	public void stop() {
		follower.followPath(new PathChain());
	}
	
	/**
	 * Implements mecanum drive using the PedroPathing Follower.
	 *
	 * @param drivePower  Forward/backward power (-1.0 to 1.0).
	 * @param strafePower Left/right strafe power (-1.0 to 1.0).
	 * @param rotation    Rotational power (-1.0 to 1.0).
	 */
	public void manual(double drivePower, double strafePower, double rotation) {
		if (state != State.MANUAL) {
			return; // Automation is handling driving, so ignore manual input.
		}
		
		double offsetHeading = robotCentric ? Math.toRadians(0) :
				MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE
						? Math.toRadians(180)
						: Math.toRadians(0);
		
		// The Follower expects a standard coordinate system (forward is positive).
		follower.setTeleOpDrive(drivePower, strafePower, -rotation, robotCentric, offsetHeading);
	}
	
	/**
	 * Moves the robot to correct for a given offset from a target (e.g., from an
	 * AprilTag).
	 * This calculates a field-centric target pose based on the robot's current pose
	 * and
	 * the robot-centric offsets, then creates and follows a path to it.
	 *
	 * @param offsetX       The robot's lateral offset from the target. Positive is
	 *                      to the right.
	 * @param offsetY       The robot's forward offset from the target. Positive is
	 *                      in front.
	 * @param offsetHeading The robot's heading offset from the target. Positive is
	 *                      clockwise.
	 */
	public void interpolateToOffset(double offsetX, double offsetY, double offsetHeading) {
		Pose currentPose = follower.getPose();
		double currentHeading = currentPose.getHeading();
		
		// Calculate the absolute target pose in the field frame.
		Pose targetPose = new Pose(
				currentPose.getX() + offsetX,
				currentPose.getY() + offsetY,
				currentHeading - offsetHeading);
		
		goTo(targetPose);
	}
	
	/**
	 * Commands the robot to follow a path to a predefined position.
	 *
	 * @param position The target position from the Position enum.
	 */
	public void goTo(Position position) {
		Pose targetPose = positionPoses.get(position);
		goTo(targetPose);
	}
	
	/**
	 * Commands the robot to follow a path to a specific field-centric pose.
	 *
	 * @param targetPose The absolute target pose.
	 */
	public void goTo(Pose targetPose) {
		if (follower.isBusy() && follower.getCurrentPath().endPose() == targetPose) {
			return;
		}
		
		this.state = State.PATHING;
		follower.holdPoint(targetPose);
	}
	
	public void rotateTo(double angle) {
		if (!follower.isTurning()) {
			
			this.state = State.PATHING;
			follower.turnTo(angle);
		}
	}
	
	/**
	 * Switches the drivetrain to manual (tele-op) control mode.
	 * This will stop any active path following.
	 */
	public void switchToManual() {
		if (state == State.MANUAL) {
			return;
		}
		this.state = State.MANUAL;
		follower.startTeleopDrive();
	}
	
	/**
	 * @return The current state of the drivetrain (MANUAL, GOTO).
	 */
	public State getState() {
		return state;
	}
	
	/**
	 * @return true if the follower is busy following a path.
	 */
	public boolean isBusy() {
		return follower.isBusy();
	}
	
	/**
	 * @return The robot's current estimated pose (x, y, heading) on the field.
	 */
	public Pose getPose() {
		return follower.getPose();
	}
	
	/**
	 * Gets the pose for a predefined position.
	 *
	 * @param position The position to get the pose for
	 * @return The pose for that position
	 */
	public Pose getPositionPose(Position position) {
		return positionPoses.get(position);
	}
	
	/**
	 * Converts a Controller.Action to a Drivetrain.Position.
	 * Returns null if the action is not a GOTO action.
	 *
	 * @param action The controller action to convert
	 * @return The corresponding Position, or null if not a GOTO action
	 */
	public Position actionToPosition(Controller.Action action) {
		switch (action) {
			case GOTO_CLOSE_SHOOT:
				return Position.CLOSE_SHOOT;
			case GOTO_FAR_SHOOT:
				return Position.FAR_SHOOT;
			case GOTO_PARK:
				return Position.PARK;
			case GOTO_GATE:
				return Position.GATE;
			default:
				return null;
		}
	}
	
	/**
	 * Checks if a Controller.Action is a GOTO action.
	 *
	 * @param action The action to check
	 * @return true if the action is a GOTO action
	 */
	public boolean isGotoAction(Controller.Action action) {
		return actionToPosition(action) != null;
	}
	
	/**
	 * Commands the robot to follow a path to a position specified by a
	 * Controller.Action.
	 * Only works for GOTO actions (GOTO_CLOSE_SHOOT, GOTO_FAR_SHOOT, GOTO_PARK,
	 * GOTO_GATE).
	 * Does nothing if the action is not a GOTO action.
	 *
	 * @param action The GOTO action specifying the target position
	 */
	public void goTo(Controller.Action action) {
		Position position = actionToPosition(action);
		if (position != null) {
			goTo(position);
		}
	}
	
	public enum Position {
		CLOSE_SHOOT,
		FAR_SHOOT,
		PARK,
		GATE,
	}
	
	public enum State {
		MANUAL,
		PATHING,
	}
}
