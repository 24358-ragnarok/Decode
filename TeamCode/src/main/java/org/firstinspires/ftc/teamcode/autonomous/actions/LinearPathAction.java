package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Path action that creates a straight line path from current position to
 * target.
 * Uses BezierLine for direct, efficient movement.
 */
public class LinearPathAction extends PathAction {
	
	public LinearPathAction(Pose targetPose, String name, MatchSettings.AllianceColor alliance) {
		super(targetPose, name, alliance);
	}
	
	public LinearPathAction(Pose targetPose, String name, MatchSettings matchSettings) {
		super(targetPose, name, matchSettings);
	}
	
	/**
	 * Convenience constructor with auto-generated name.
	 */
	public LinearPathAction(Pose targetPose, MatchSettings matchSettings) {
		super(targetPose, "LinearPath", matchSettings);
	}
	
	@Override
	protected PathChain buildPath(MechanismManager mechanisms, Pose startPose, Pose endPose) {
		return mechanisms.drivetrain.follower.pathBuilder()
				.addPath(new BezierLine(startPose, endPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
				.build();
	}
}
