package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Abstract base class for all path-following actions.
 * Handles dynamic path building from current robot position to target poses,
 * with automatic alliance mirroring and different path types.
 * <p>
 * This replaces the rigid PathRegistry system with flexible, composable
 * actions.
 */
public abstract class PathAction implements AutonomousAction {
	protected final Pose targetPose;
	protected final String name;
	protected final MatchState.AllianceColor alliance;
	
	private PathChain generatedPath;
	
	/**
	 * Creates a new path action.
	 *
	 * @param targetPose The target pose (in BLUE alliance coordinates)
	 * @param name       Human-readable name for telemetry
	 * @param alliance   The alliance color for automatic mirroring
	 */
	public PathAction(Pose targetPose, String name, MatchState.AllianceColor alliance) {
		this.targetPose = targetPose;
		this.name = name;
		this.alliance = alliance;
	}
	
	/**
	 * Convenience constructor that extracts alliance from MatchState.
	 */
	public PathAction(Pose targetPose, String name) {
		this(targetPose, name, MatchState.getAllianceColor());
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		// Get current robot position
		Pose currentPose = mechanisms.bentDrivetrain.follower.getPose();
		
		// Mirror target pose if we're on RED alliance
		Pose actualTarget = (alliance == MatchState.AllianceColor.BLUE)
				? targetPose
				: Settings.Field.mirrorPose(targetPose);
		
		// Build the path dynamically from current position to target
		generatedPath = buildPath(mechanisms, currentPose, actualTarget);
		
		// Start following the path
		followPath(mechanisms, generatedPath);
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// Action is complete when the follower is no longer busy
		return !mechanisms.bentDrivetrain.follower.isBusy();
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// Path following will naturally stop when complete
		// If interrupted, break
		if (interrupted) {
			mechanisms.bentDrivetrain.follower.breakFollowing();
		}
	}
	
	@Override
	public String getName() {
		return name;
	}
	
	/**
	 * Builds the actual path from start to end pose.
	 * Subclasses implement this to create different path types (linear, curved,
	 * etc.).
	 *
	 * @param mechanisms The mechanism manager
	 * @param startPose  The starting pose (robot's current position)
	 * @param endPose    The target pose (already mirrored for alliance)
	 * @return The constructed PathChain
	 */
	protected abstract PathChain buildPath(MechanismManager mechanisms, Pose startPose, Pose endPose);
	
	/**
	 * Starts following the generated path.
	 * Subclasses can override this to modify following behavior (e.g., slow speed).
	 *
	 * @param mechanisms The mechanism manager
	 * @param path       The path to follow
	 */
	protected void followPath(MechanismManager mechanisms, PathChain path) {
		mechanisms.bentDrivetrain.follower.followPath(path, true);
	}
	
	/**
	 * Gets the final target pose (after alliance mirroring).
	 * Useful for debugging and telemetry.
	 */
	public Pose getFinalTargetPose() {
		return (alliance == MatchState.AllianceColor.BLUE)
				? targetPose
				: Settings.Field.mirrorPose(targetPose);
	}
}
