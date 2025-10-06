package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.CLOSE_LAUNCH_ZONE_FRONT_CORNER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.CLOSE_LAUNCH_ZONE_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.CLOSE_LAUNCH_ZONE_RIGHT_CORNER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.FAR_LAUNCH_ZONE_FRONT_CORNER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.FAR_LAUNCH_ZONE_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Field.FAR_LAUNCH_ZONE_RIGHT_CORNER;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

/**
 * The AlignmentEngine aligns the robot chassis during aiming. This is fully
 * decoupled from the
 * {@link TrajectoryEngine} such that they move independently of each other;
 * this means if we get
 * pushed by another robot, the launcher will maintain the angle of the shot by
 * rotating the launcher component,
 * while the alignment engine realigns the chassis; both try to point in the
 * direction of the goal.
 * This allows the AE to fix large yaw errors while the TE fixes small yaw and
 * pitch errors
 * with both systems working in decoupled realtime tandem.
 */
public class AlignmentEngine extends Mechanism {
	private final Drivetrain drivetrain;
	private final MatchSettings matchSettings;
	@SuppressWarnings("FieldCanBeLocal") // Reserved for future visual alignment features
	private final LimelightManager limelightManager;
	
	public AlignmentEngine(MatchSettings matchSettings, Drivetrain drivetrain, LimelightManager limelightManager) {
		this.drivetrain = drivetrain;
		this.matchSettings = matchSettings;
		this.limelightManager = limelightManager;
	}
	
	/**
	 * Checks if a point is inside a triangle.
	 *
	 * @param pose The point to check
	 * @param A    The first vertex of the triangle
	 * @param B    The second vertex of the triangle
	 * @param C    The third vertex of the triangle
	 * @return True if the point is inside the triangle, false otherwise
	 */
	public static boolean isInsideTriangle(Pose pose, Pose A, Pose B, Pose C) {
		// Create new Point objects for the pose's x and y
		Pose P = new Pose(pose.getX(), pose.getY());
		
		// Use the crossProduct method to determine if the point is within the triangle
		double s1 = crossProduct(A, B, P);
		double s2 = crossProduct(B, C, P);
		double s3 = crossProduct(C, A, P);
		
		boolean has_neg = (s1 < 0) || (s2 < 0) || (s3 < 0);
		boolean has_pos = (s1 > 0) || (s2 > 0) || (s3 > 0);
		
		return !(has_neg && has_pos);
	}
	
	/**
	 * Calculates the cross product for point-in-triangle testing.
	 * Reference: <a href="https://stackoverflow.com/questions/2049582">...</a>
	 */
	public static double crossProduct(Pose A, Pose B, Pose C) {
		return (B.getX() - A.getX()) * (C.getY() - A.getY()) - (B.getY() - A.getY()) * (C.getX() - A.getX());
	}
	
	public void init() {
		// No initialization required for this mechanism
	}
	
	/**
	 * Checks if the robot is aligned with the goal.
	 *
	 * @return True if the robot is aligned with the goal, false otherwise
	 */
	public boolean isAligned() {
		Pose currentPose = drivetrain.follower.getPose();
		Pose targetPose = (matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE)
				? Settings.Field.BLUE_GOAL_POSE
				: Settings.Field.RED_GOAL_POSE;
		
		double angleError = angleToTarget(currentPose, targetPose);
		return Math.abs(angleError) < Settings.Aiming.MAX_ROTATIONAL_ERROR;
	}
	
	public void run() {
		Pose currentPose = drivetrain.follower.getPose();
		
		Pose newPose = currentPose.copy()
				.withX(Math.round(currentPose.getX()))
				.withY(Math.round(currentPose.getY()));
		
		Pose targetPose = (matchSettings.getAllianceColor() == MatchSettings.AllianceColor.BLUE)
				? Settings.Field.BLUE_GOAL_POSE
				: Settings.Field.RED_GOAL_POSE;
		
		double angleError = angleToTarget(newPose, targetPose);
		
		drivetrain.goTo(newPose.withHeading(angleError));
	}
	
	public void update() {
	}
	
	@Override
	public void stop() {
	
	}
	
	public boolean isInLaunchZone(Pose pose) {
		// Check if the pose is inside the FAR launch zone
		boolean inFarZone = isInsideTriangle(
				pose,
				FAR_LAUNCH_ZONE_FRONT_CORNER,
				FAR_LAUNCH_ZONE_LEFT_CORNER,
				FAR_LAUNCH_ZONE_RIGHT_CORNER);
		
		// Check if the pose is inside the CLOSE launch zone
		boolean inCloseZone = isInsideTriangle(
				pose,
				CLOSE_LAUNCH_ZONE_FRONT_CORNER,
				CLOSE_LAUNCH_ZONE_LEFT_CORNER,
				CLOSE_LAUNCH_ZONE_RIGHT_CORNER);
		
		// Return true if the pose is in *either* launch zone
		return inFarZone || inCloseZone;
	}
	
	// returns signed smallest angle to face target
	public double angleToTarget(Pose currentPose, Pose targetPose) {
		double dx = targetPose.getX() - currentPose.getX();
		double dy = targetPose.getY() - currentPose.getY();
		
		double desired = Math.atan2(dy, dx); // absolute angle to goal
		
		while (desired > Math.PI)
			desired -= 2 * Math.PI;
		while (desired <= -Math.PI)
			desired += 2 * Math.PI;
		return desired; // radians, positive -> rotate CCW, negative -> rotate CW
	}
}
