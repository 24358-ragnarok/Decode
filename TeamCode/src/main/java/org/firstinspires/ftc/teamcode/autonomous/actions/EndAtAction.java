package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Action that follows a PathChain using the bentDrivetrain's path follower.
 */
public class EndAtAction implements AutonomousAction {
	private final Pose pose;
	private final String name;
	
	public EndAtAction(Pose pose, String name) {
		this.pose = pose;
		this.name = name;
	}
	
	public EndAtAction(Pose pose) {
		this(pose, "EndAt");
	}
	
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		mechanisms.bentDrivetrain.follower.holdPoint(pose);
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// End actions never complete
		return false;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// never ends
	}
	
	@Override
	public String getName() {
		return name;
	}
}
