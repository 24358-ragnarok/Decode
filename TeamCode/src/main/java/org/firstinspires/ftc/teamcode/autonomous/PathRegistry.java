package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;

import java.util.ArrayList;

/**
 * Centralized registry for all autonomous paths with automatic mirroring.
 * <p>
 * This class eliminates duplication by:
 * - Defining paths once for RED alliance
 * - Automatically mirroring them for BLUE alliance
 * - Using descriptive enums instead of multiple variables
 * - Centralizing path construction logic
 */
public class PathRegistry {
	
	private final Follower follower;
	private final MatchSettings.AllianceColor alliance;
	private final MatchSettings.AutoStartingPosition position;
	
	public PathRegistry(Follower follower, MatchSettings matchSettings) {
		this.follower = follower;
		this.alliance = matchSettings.getAllianceColor();
		this.position = matchSettings.getAutoStartingPosition();
	}
	
	/**
	 * Gets a path by its segment identifier.
	 * Automatically handles alliance-based mirroring.
	 *
	 * @param segment The path segment to retrieve
	 * @return The constructed PathChain
	 */
	public PathChain getPath(PathSegment segment) {
		if (position == MatchSettings.AutoStartingPosition.FAR) {
			return buildFarPath(segment);
		} else {
			return buildClosePath(segment);
		}
	}
	
	/**
	 * Builds a far position path.
	 * Uses Settings for RED, mirrors for BLUE.
	 */
	private PathChain buildFarPath(PathSegment segment) {
		boolean isRed = alliance == MatchSettings.AllianceColor.RED;
		
		switch (segment) {
			case FAR_PRESET_1_PREP:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedFar.START : mirror(Settings.Autonomous.RedFar.START),
						isRed ? Settings.Autonomous.RedFar.PRESET_1_PREP
								: mirror(Settings.Autonomous.RedFar.PRESET_1_PREP));
			
			case FAR_PRESET_1_END:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedFar.PRESET_1_PREP
								: mirror(Settings.Autonomous.RedFar.PRESET_1_PREP),
						isRed ? Settings.Autonomous.RedFar.PRESET_1_END
								: mirror(Settings.Autonomous.RedFar.PRESET_1_END));
			
			case FAR_LAUNCH_1:
				if (isRed) {
					return follower.pathBuilder()
							.addPath(Settings.Autonomous.RedFar.BEZIER_LAUNCH_1)
							.setLinearHeadingInterpolation(
									Settings.Autonomous.RedFar.PRESET_1_END.getHeading(),
									Settings.Autonomous.RedFar.ENDING_LAUNCH_1.getHeading())
							.build();
				} else {
					BezierCurve mirrored = mirrorBezierCurve(Settings.Autonomous.RedFar.BEZIER_LAUNCH_1);
					return follower.pathBuilder()
							.addPath(mirrored)
							.setLinearHeadingInterpolation(
									mirror(Settings.Autonomous.RedFar.PRESET_1_END).getHeading(),
									mirror(Settings.Autonomous.RedFar.ENDING_LAUNCH_1).getHeading())
							.build();
				}
			
			case FAR_PRESET_2_PREP:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedFar.ENDING_LAUNCH_1
								: mirror(Settings.Autonomous.RedFar.ENDING_LAUNCH_1),
						isRed ? Settings.Autonomous.RedFar.PRESET_2_PREP
								: mirror(Settings.Autonomous.RedFar.PRESET_2_PREP));
			
			case FAR_PRESET_2_END:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedFar.PRESET_2_PREP
								: mirror(Settings.Autonomous.RedFar.PRESET_2_PREP),
						isRed ? Settings.Autonomous.RedFar.PRESET_2_END
								: mirror(Settings.Autonomous.RedFar.PRESET_2_END));
			
			case FAR_LAUNCH_2:
				return buildCurvedPath(
						isRed ? Settings.Autonomous.RedFar.PRESET_2_END
								: mirror(Settings.Autonomous.RedFar.PRESET_2_END),
						isRed ? Settings.Autonomous.RedFar.LAUNCH_2 : mirror(Settings.Autonomous.RedFar.LAUNCH_2));
			
			case FAR_PRESET_3:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedFar.LAUNCH_2 : mirror(Settings.Autonomous.RedFar.LAUNCH_2),
						isRed ? Settings.Autonomous.RedFar.PRESET_3_END
								: mirror(Settings.Autonomous.RedFar.PRESET_3_END));
			
			case FAR_LAUNCH_3:
				return buildCurvedPath(
						isRed ? Settings.Autonomous.RedFar.PRESET_3_END
								: mirror(Settings.Autonomous.RedFar.PRESET_3_END),
						isRed ? Settings.Autonomous.RedFar.PARK : mirror(Settings.Autonomous.RedFar.PARK));
			
			case FAR_PARK:
				Pose park = isRed ? Settings.Autonomous.RedFar.PARK : mirror(Settings.Autonomous.RedFar.PARK);
				return buildLinearPath(park, park);
			
			default:
				throw new IllegalArgumentException("Invalid far path segment: " + segment);
		}
	}
	
	/**
	 * Builds a close position path.
	 * Uses Settings for RED, mirrors for BLUE.
	 */
	private PathChain buildClosePath(PathSegment segment) {
		boolean isRed = alliance == MatchSettings.AllianceColor.RED;
		
		switch (segment) {
			case CLOSE_PRESET_1_PREP:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.START : mirror(Settings.Autonomous.RedClose.START),
						isRed ? Settings.Autonomous.RedClose.PRESET_1_PREP
								: mirror(Settings.Autonomous.RedClose.PRESET_1_PREP));
			
			case CLOSE_PRESET_1_END:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.PRESET_1_PREP
								: mirror(Settings.Autonomous.RedClose.PRESET_1_PREP),
						isRed ? Settings.Autonomous.RedClose.PRESET_1_END
								: mirror(Settings.Autonomous.RedClose.PRESET_1_END));
			
			case CLOSE_LAUNCH_1:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.PRESET_1_END
								: mirror(Settings.Autonomous.RedClose.PRESET_1_END),
						isRed ? Settings.Autonomous.RedClose.LAUNCH_1 : mirror(Settings.Autonomous.RedClose.LAUNCH_1));
			
			case CLOSE_PRESET_2_PREP:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.LAUNCH_1 : mirror(Settings.Autonomous.RedClose.LAUNCH_1),
						isRed ? Settings.Autonomous.RedClose.PRESET_2_PREP
								: mirror(Settings.Autonomous.RedClose.PRESET_2_PREP));
			
			case CLOSE_PRESET_2_END:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.PRESET_2_PREP
								: mirror(Settings.Autonomous.RedClose.PRESET_2_PREP),
						isRed ? Settings.Autonomous.RedClose.PRESET_2_END
								: mirror(Settings.Autonomous.RedClose.PRESET_2_END));
			
			case CLOSE_LAUNCH_2:
				return buildCurvedPath(
						isRed ? Settings.Autonomous.RedClose.PRESET_2_END
								: mirror(Settings.Autonomous.RedClose.PRESET_2_END),
						isRed ? Settings.Autonomous.RedClose.LAUNCH_2 : mirror(Settings.Autonomous.RedClose.LAUNCH_2));
			
			case CLOSE_PRESET_3_PREP:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.LAUNCH_2 : mirror(Settings.Autonomous.RedClose.LAUNCH_2),
						isRed ? Settings.Autonomous.RedClose.PRESET_3_PREP
								: mirror(Settings.Autonomous.RedClose.PRESET_3_PREP));
			
			case CLOSE_PRESET_3_END:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.PRESET_3_PREP
								: mirror(Settings.Autonomous.RedClose.PRESET_3_PREP),
						isRed ? Settings.Autonomous.RedClose.PRESET_3_END
								: mirror(Settings.Autonomous.RedClose.PRESET_3_END));
			
			case CLOSE_LAUNCH_3:
				return buildCurvedPath(
						isRed ? Settings.Autonomous.RedClose.PRESET_3_END
								: mirror(Settings.Autonomous.RedClose.PRESET_3_END),
						isRed ? Settings.Autonomous.RedClose.LAUNCH_3 : mirror(Settings.Autonomous.RedClose.LAUNCH_3));
			
			case CLOSE_PARK:
				return buildLinearPath(
						isRed ? Settings.Autonomous.RedClose.LAUNCH_3 : mirror(Settings.Autonomous.RedClose.LAUNCH_3),
						isRed ? Settings.Autonomous.RedClose.PARK : mirror(Settings.Autonomous.RedClose.PARK));
			
			default:
				throw new IllegalArgumentException("Invalid close path segment: " + segment);
		}
	}
	
	/**
	 * Builds a linear path between two poses.
	 */
	private PathChain buildLinearPath(Pose start, Pose end) {
		return follower.pathBuilder()
				.addPath(new BezierLine(start, end))
				.setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
				.build();
	}
	
	/**
	 * Builds a curved path between two poses.
	 */
	private PathChain buildCurvedPath(Pose start, Pose end) {
		return follower.pathBuilder()
				.addPath(new BezierCurve(start, end))
				.setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
				.build();
	}
	
	/**
	 * Mirrors a pose across the field centerline for blue alliance.
	 * Field width is 144 inches (standard FTC field).
	 */
	private Pose mirror(Pose redPose) {
		double fieldWidth = 144.0;
		return new Pose(
				fieldWidth - redPose.getX(), // Mirror X coordinate
				redPose.getY(), // Y stays the same
				Math.PI - redPose.getHeading() // Mirror heading
		);
	}
	
	/**
	 * Mirrors a BezierCurve for blue alliance.
	 */
	private BezierCurve mirrorBezierCurve(BezierCurve curve) {
		// Get the control points from the curve and mirror each one
		ArrayList<Pose> controlPoints = curve.getControlPoints();
		Pose[] mirroredPoints = new Pose[controlPoints.size()];
		
		for (int i = 0; i < controlPoints.size(); i++) {
			mirroredPoints[i] = mirror(controlPoints.get(i));
		}
		
		return new BezierCurve(mirroredPoints);
	}
	
	/**
	 * Enum defining all possible path segments in the autonomous routine.
	 */
	public enum PathSegment {
		// Far position paths
		FAR_PRESET_1_PREP,
		FAR_PRESET_1_END,
		FAR_LAUNCH_1,
		FAR_PRESET_2_PREP,
		FAR_PRESET_2_END,
		FAR_LAUNCH_2,
		FAR_PRESET_3,
		FAR_LAUNCH_3,
		FAR_PARK,
		
		// Close position paths
		CLOSE_PRESET_1_PREP,
		CLOSE_PRESET_1_END,
		CLOSE_LAUNCH_1,
		CLOSE_PRESET_2_PREP,
		CLOSE_PRESET_2_END,
		CLOSE_LAUNCH_2,
		CLOSE_PRESET_3_PREP,
		CLOSE_PRESET_3_END,
		CLOSE_LAUNCH_3,
		CLOSE_PARK
	}
}
