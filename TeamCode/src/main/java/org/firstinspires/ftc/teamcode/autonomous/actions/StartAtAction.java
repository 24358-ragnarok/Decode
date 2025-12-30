package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Initialization action that locks the drivetrain to a starting pose.
 * Sets a hold point immediately to eliminate race conditions between
 * pose initialization and the first movement action.
 */
public class StartAtAction implements AutonomousAction {
	private final Pose targetPose;
	private final String name;
	private final MatchState.AllianceColor alliance;
	
	public StartAtAction(Pose targetPose, String name, MatchState.AllianceColor alliance) {
		this.targetPose = targetPose;
		this.name = name;
		this.alliance = alliance;
	}
	
	public StartAtAction(Pose targetPose, String name) {
		this(targetPose, name, MatchState.getAllianceColor());
	}
	
	public StartAtAction(Pose targetPose) {
		this(targetPose, "StartAt");
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		Pose actualPose = (alliance == MatchState.AllianceColor.BLUE)
				? targetPose
				: Settings.Field.mirrorPose(targetPose);
		
		// Ensure follower starts at the desired pose and holds position
		mechanisms.drivetrain.follower.setStartingPose(actualPose);
		mechanisms.drivetrain.follower.holdPoint(actualPose);
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// Nothing to wait on; move to next action immediately.
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// No cleanup required; holdPoint maintains position if still running.
	}
	
	@Override
	public String getName() {
		return name;
	}
}
