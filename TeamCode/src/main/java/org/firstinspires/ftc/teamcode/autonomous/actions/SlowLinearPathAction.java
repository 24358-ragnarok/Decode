package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Path action that creates a straight line path with reduced speed.
 * Useful for precise movements like sample pickup.
 */
public class SlowLinearPathAction extends PathAction {
	
	public SlowLinearPathAction(Pose targetPose, String name, MatchState.AllianceColor alliance) {
		super(targetPose, name, alliance);
	}
	
	public SlowLinearPathAction(Pose targetPose, String name) {
		super(targetPose, name);
	}

	/**
	 * Convenience constructor with auto-generated name.
	 */
	public SlowLinearPathAction(Pose targetPose) {
		super(targetPose, "SlowLinearPath");
	}
	
	@Override
	protected PathChain buildPath(MechanismManager mechanisms, Pose startPose, Pose endPose) {
		return mechanisms.drivetrain.follower.pathBuilder()
				.addPath(new BezierLine(startPose, endPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
				.build();
	}
	
	@Override
	protected void followPath(MechanismManager mechanisms, PathChain path) {
		// Use reduced speed for precise movement
		mechanisms.drivetrain.follower.followPath(path, Settings.Autonomous.SLOW_SPEED, true);
	}
}
