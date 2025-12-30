package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Path action that creates a curved path from current position to target using
 * control points.
 * Builds curves dynamically like LinearPathAction, maintaining path
 * consistency.
 * Uses BezierCurve with intermediate control points for smooth, tuned
 * movements.
 */
public class CurvePathAction extends PathAction {
	
	private final Pose[] controlPoints;
	
	/**
	 * Creates a curve path action with explicit control points.
	 * The curve will be built dynamically from current position → control points →
	 * target.
	 *
	 * @param targetPose    The target pose (in BLUE alliance coordinates)
	 * @param name          Human-readable name for telemetry
	 * @param alliance      The alliance color for automatic mirroring
	 * @param controlPoints Intermediate control points for the curve (in BLUE
	 *                      coordinates)
	 */
	public CurvePathAction(Pose targetPose, String name, MatchState.AllianceColor alliance, Pose... controlPoints) {
		super(targetPose, name, alliance);
		this.controlPoints = controlPoints;
	}
	
	public CurvePathAction(Pose targetPose, String name, Pose... controlPoints) {
		super(targetPose, name);
		this.controlPoints = controlPoints;
	}
	
	/**
	 * Convenience constructor with auto-generated name.
	 */
	public CurvePathAction(Pose targetPose, Pose... controlPoints) {
		super(targetPose, "CurvePath");
		this.controlPoints = controlPoints;
	}
	
	/**
	 * Creates a curve with a single control point (simple 3-point curve).
	 */
	public static CurvePathAction withSingleControlPoint(Pose targetPose, Pose controlPoint, String name) {
		return new CurvePathAction(targetPose, name, controlPoint);
	}
	
	@Override
	protected PathChain buildPath(MechanismManager mechanisms, Pose startPose, Pose endPose) {
		// Build curve dynamically from current position
		Pose[] allPoints = new Pose[controlPoints.length + 2];
		allPoints[0] = startPose; // Always start from current position
		
		// Add control points (mirror them if RED alliance)
		for (int i = 0; i < controlPoints.length; i++) {
			allPoints[i + 1] = (alliance == MatchState.AllianceColor.BLUE)
					? controlPoints[i]
					: Settings.Field.mirrorPose(controlPoints[i]);
		}
		
		allPoints[allPoints.length - 1] = endPose; // End at target (already mirrored by parent)
		
		BezierCurve curve = new BezierCurve(allPoints);
		
		return mechanisms.drivetrain.follower.pathBuilder()
				.addPath(curve)
				.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
				.build();
	}
}
