package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Action that follows a PathChain using the drivetrain's path follower.
 */
public class FollowPathAction implements AutonomousAction {
	private final PathChain path;
	private final String name;
	private final boolean slowly;
	
	public FollowPathAction(PathChain path, String name, boolean slowly) {
		this.path = path;
		this.name = name;
		this.slowly = slowly;
	}
	
	public FollowPathAction(PathChain path) {
		this(path, "FollowPath", false);
	}
	
	public FollowPathAction(PathChain path, boolean slowly) {
		this(path, "FollowPath", slowly);
	}
	
	public FollowPathAction(PathChain path, String name) {
		this(path, name, false);
	}
	
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		if (slowly) {
			mechanisms.drivetrain.follower.followPath(path, 0.23, true);
		} else {
			mechanisms.drivetrain.follower.followPath(path);
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// Action is complete when the follower is no longer busy
		return !mechanisms.drivetrain.follower.isBusy();
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// Path following will naturally stop when complete
		// If interrupted, the next action will handle it
	}
	
	@Override
	public String getName() {
		return name;
	}
}
