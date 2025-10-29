package org.firstinspires.ftc.teamcode.software;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
	// TODO: Tune these coordinates for your actual field and alliance.
	private final Map<Position, Pose> positionPoses = new HashMap<>();
	public boolean robotCentric = true;
	private State state;
	
	/**
	 * Initializes the Drivetrain and the PedroPathing Follower.
	 * <p>
	 * Note: Starting pose should be set by the OpMode (MainAuto or MainOp),
	 * not in this constructor.
	 *
	 * @param hardwareMap The robot's hardware map.
	 */
	public Drivetrain(HardwareMap hardwareMap, MatchSettings matchSettings) {
		// The Constants class now holds all hardware and tuning configurations.
		this.follower = Constants.createFollower(hardwareMap);
		// Don't set starting pose here - let each OpMode handle it
		
		// Initialize the poses for each predefined position
		positionPoses.put(Position.CLOSE_SHOOT, new Pose(60, 89, Math.toRadians(115)));
		positionPoses.put(Position.FAR_SHOOT, new Pose(60, 15, Math.toRadians(115)));
		positionPoses.put(Position.HUMAN_PLAYER, new Pose(30, 30, Math.toRadians(225)));
		positionPoses.put(Position.GATE, new Pose(25, 68, Math.toRadians(0)));
	}
	
	@Override
	public void init() {
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
		
		// When an automated movement (GOTO) is finished,
		// automatically switch back to manual control.
		if ((state == State.PATHING) && !follower.isBusy()) {
			switchToManual();
		}
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
	public void manual(double drivePower, double strafePower, double rotation, double offsetHeading) {
		if (state != State.MANUAL) {
			return; // Automation is handling driving, so ignore manual input.
		}
		if (robotCentric) {
			offsetHeading = Math.toRadians(0); // do not rotate movement if now
		}
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
		this.state = State.PATHING;
		PathChain path = follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, targetPose)))
				.setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
				.build();
		follower.followPath(path);
	}
	
	public void rotateTo(double angle) {
		this.state = State.PATHING;
		
		if (!follower.isBusy() || Math.round(follower.getCurrentPath().getHeadingGoal(1)) != Math.round(angle)) {
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
	
	public enum Position {
		CLOSE_SHOOT,
		FAR_SHOOT,
		HUMAN_PLAYER,
		GATE,
	}
	
	public enum State {
		MANUAL,
		PATHING,
	}
}
