package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.PairedLauncher;
import org.firstinspires.ftc.teamcode.software.LimelightManager;
import org.firstinspires.ftc.teamcode.software.game.Classifier;
import org.firstinspires.ftc.teamcode.software.game.Motif;

/**
 * Action that scans the obelisk during autonomous.
 * <p>
 * This action coordinates both the drivetrain and limelight to point towards the obelisk, check the obelisk, and then continue.
 */
public class ScanAction implements AutonomousAction {
	private final boolean turn;
	
	public ScanAction(boolean turn) {
		this.turn = turn;
	}
	
	public ScanAction() {
		this(false);
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		// 1. Get the current robot pose
		Pose currentPose = mechanisms.drivetrain.follower.getPose();
		
		// 2. Get the target pose (Obelisk position)
		Pose targetPose = Settings.Positions.Towers.OBELISK;
		
		// 3. Calculate the difference in X and Y coordinates
		double dx = targetPose.getX() - currentPose.getX();
		double dy = targetPose.getY() - currentPose.getY();
		
		// 4. Use Math.atan2(dy, dx) to find the angle in radians
		//    from currentPose to targetPose, relative to the positive X-axis.
		//    atan2 handles all four quadrants correctly.
		double targetAngle = Math.atan2(dy, dx);
		
		// Check if the action should perform the turn (default is true)
		if (turn) {
			// Use the calculated angle to turn the robot
			mechanisms.drivetrain.follower.turnTo(targetAngle);
		}
		
		// begin scanning
		mechanisms.ifValid(mechanisms.limelightManager, limelightManager ->
				limelightManager.setCurrentPipeline(LimelightManager.Pipeline.APRILTAG));
		
		mechanisms.ifValid(mechanisms.get(PairedLauncher.class), launcher -> {
			launcher.setPitch(1);
		});
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		if (!Settings.Deploy.LIMELIGHT) {
			return true;
		}
		Motif m = mechanisms.limelightManager.detectMotif();
		if (m == Motif.UNKNOWN) {
			return false;
		}
		MatchState.setClassifier(new Classifier(m));
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		if (interrupted) {
			mechanisms.drivetrain.follower.breakFollowing();
		}
	}
	
	@Override
	public String getName() {
		return "Scan " + (turn ? "(turn)" : "(stationary)");
	}
}
