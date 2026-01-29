package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Path action that creates a smooth curved path using a Bezier curve.
 * Automatically generates control points for natural movement.
 */
public class SplinedPathAction extends PathAction {
	
	private final Pose[] controlPoints;
	
	/**
	 * Creates a splined path with explicit control points.
	 *
	 * @param targetPose    The target pose (in BLUE alliance coordinates)
	 * @param name          Human-readable name for telemetry
	 * @param alliance      The alliance color for automatic mirroring
	 * @param controlPoints Additional control points for the curve (in BLUE
	 *                      coordinates)
	 */
	public SplinedPathAction(Pose targetPose, String name, MatchState.AllianceColor alliance,
	                         Pose... controlPoints) {
		super(targetPose, name, alliance);
		this.controlPoints = controlPoints;
	}
	
	public SplinedPathAction(Pose targetPose, String name, Pose... controlPoints) {
		super(targetPose, name);
		this.controlPoints = controlPoints;
	}
	
	/**
	 * Creates a splined path with auto-generated control points.
	 * The control point is placed at the midpoint between start and end.
	 */
	public SplinedPathAction(Pose targetPose, String name) {
		super(targetPose, name);
		this.controlPoints = null; // Will be auto-generated
	}
	
	/**
	 * Convenience constructor with auto-generated name and control points.
	 */
	public SplinedPathAction(Pose targetPose) {
		super(targetPose, "SplinedPath");
		this.controlPoints = null; // Will be auto-generated
	}
	
	@Override
	protected PathChain buildPath(MechanismManager mechanisms, Pose startPose, Pose endPose) {
		BezierCurve curve;
		
		if (controlPoints != null && controlPoints.length > 0) {
			// Use explicit control points (mirror them if needed)
			Pose[] allPoints = new Pose[controlPoints.length + 2];
			allPoints[0] = startPose;
			
			for (int i = 0; i < controlPoints.length; i++) {
				allPoints[i + 1] = (alliance == MatchState.AllianceColor.BLUE)
						? controlPoints[i]
						: Settings.Field.mirrorPose(controlPoints[i]);
			}
			
			allPoints[allPoints.length - 1] = endPose;
			curve = new BezierCurve(allPoints);
		} else {
			// Auto-generate a simple curve with midpoint control
			Pose midPoint = new Pose(
					(startPose.getX() + endPose.getX()) / 2,
					(startPose.getY() + endPose.getY()) / 2,
					(startPose.getHeading() + endPose.getHeading()) / 2);
			curve = new BezierCurve(startPose, midPoint, endPose);
		}
		
		return mechanisms.drivetrain.follower.pathBuilder()
				.addPath(curve)
				.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
				.build();
	}
}
