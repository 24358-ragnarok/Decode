package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.hardware.FlexVectorIntake;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Path action that creates a straight line path from current position to
 * target.
 * Uses BezierLine for direct, efficient movement.
 */
public class SafePickupPathAction extends PathAction {
	
	public SafePickupPathAction(Pose targetPose, String name, MatchState.AllianceColor alliance) {
		super(targetPose, name, alliance);
	}
	
	public SafePickupPathAction(Pose targetPose, String name) {
		super(targetPose, name);
	}
	
	/**
	 * Convenience constructor with auto-generated name.
	 */
	public SafePickupPathAction(Pose targetPose) {
		super(targetPose, "LinearPath");
	}
	
	@Override
	protected PathChain buildPath(MechanismManager mechanisms, Pose startPose, Pose endPose) {
		return mechanisms.drivetrain.follower.pathBuilder()
				.addPath(new BezierLine(startPose, endPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
				.addParametricCallback(0.4, () -> mechanisms.ifValid(mechanisms.get(FlexVectorIntake.class), FlexVectorIntake::stop))
				.build();
	}
}
