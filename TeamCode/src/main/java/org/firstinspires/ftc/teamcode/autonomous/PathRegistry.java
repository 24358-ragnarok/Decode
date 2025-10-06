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
 * - Defining paths once for BLUE alliance (our reference field)
 * - Automatically mirroring them for RED alliance
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
	 * Uses Settings for BLUE (reference), mirrors for RED.
	 */
	private PathChain buildFarPath(PathSegment segment) {
		boolean isBlue = alliance == MatchSettings.AllianceColor.BLUE;
		
		switch (segment) {
			case FAR_PRESET_1_PREP:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueFar.START : mirror(Settings.Autonomous.BlueFar.START),
						isBlue ? Settings.Autonomous.BlueFar.PRESET_1_PREP
								: mirror(Settings.Autonomous.BlueFar.PRESET_1_PREP));
			
			case FAR_PRESET_1_END:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueFar.PRESET_1_PREP
								: mirror(Settings.Autonomous.BlueFar.PRESET_1_PREP),
						isBlue ? Settings.Autonomous.BlueFar.PRESET_1_END
								: mirror(Settings.Autonomous.BlueFar.PRESET_1_END));
			
			case FAR_LAUNCH_1:
				if (isBlue) {
					return follower.pathBuilder()
							.addPath(Settings.Autonomous.BlueFar.BEZIER_LAUNCH_1)
							.setLinearHeadingInterpolation(
									Settings.Autonomous.BlueFar.PRESET_1_END.getHeading(),
									Settings.Autonomous.BlueFar.ENDING_LAUNCH_1.getHeading())
							.build();
				} else {
					BezierCurve mirrored = mirrorBezierCurve(Settings.Autonomous.BlueFar.BEZIER_LAUNCH_1);
					return follower.pathBuilder()
							.addPath(mirrored)
							.setLinearHeadingInterpolation(
									mirror(Settings.Autonomous.BlueFar.PRESET_1_END).getHeading(),
									mirror(Settings.Autonomous.BlueFar.ENDING_LAUNCH_1).getHeading())
							.build();
				}
			
			case FAR_PRESET_2_PREP:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueFar.ENDING_LAUNCH_1
								: mirror(Settings.Autonomous.BlueFar.ENDING_LAUNCH_1),
						isBlue ? Settings.Autonomous.BlueFar.PRESET_2_PREP
								: mirror(Settings.Autonomous.BlueFar.PRESET_2_PREP));
			
			case FAR_PRESET_2_END:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueFar.PRESET_2_PREP
								: mirror(Settings.Autonomous.BlueFar.PRESET_2_PREP),
						isBlue ? Settings.Autonomous.BlueFar.PRESET_2_END
								: mirror(Settings.Autonomous.BlueFar.PRESET_2_END));
			
			case FAR_LAUNCH_2:
				return buildCurvedPath(
						isBlue ? Settings.Autonomous.BlueFar.PRESET_2_END
								: mirror(Settings.Autonomous.BlueFar.PRESET_2_END),
						isBlue ? Settings.Autonomous.BlueFar.LAUNCH_2 : mirror(Settings.Autonomous.BlueFar.LAUNCH_2));
			
			case FAR_PRESET_3:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueFar.LAUNCH_2 : mirror(Settings.Autonomous.BlueFar.LAUNCH_2),
						isBlue ? Settings.Autonomous.BlueFar.PRESET_3_END
								: mirror(Settings.Autonomous.BlueFar.PRESET_3_END));
			
			case FAR_LAUNCH_3:
				return buildCurvedPath(
						isBlue ? Settings.Autonomous.BlueFar.PRESET_3_END
								: mirror(Settings.Autonomous.BlueFar.PRESET_3_END),
						isBlue ? Settings.Autonomous.BlueFar.PARK : mirror(Settings.Autonomous.BlueFar.PARK));
			
			case FAR_PARK:
				Pose park = isBlue ? Settings.Autonomous.BlueFar.PARK : mirror(Settings.Autonomous.BlueFar.PARK);
				return buildLinearPath(park, park);
			
			default:
				throw new IllegalArgumentException("Invalid far path segment: " + segment);
		}
	}
	
	/**
	 * Builds a close position path.
	 * Uses Settings for BLUE (reference), mirrors for RED.
	 */
	private PathChain buildClosePath(PathSegment segment) {
		boolean isBlue = alliance == MatchSettings.AllianceColor.BLUE;
		
		switch (segment) {
			case CLOSE_PRESET_1_PREP:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.START : mirror(Settings.Autonomous.BlueClose.START),
						isBlue ? Settings.Autonomous.BlueClose.PRESET_1_PREP
								: mirror(Settings.Autonomous.BlueClose.PRESET_1_PREP));
			
			case CLOSE_PRESET_1_END:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.PRESET_1_PREP
								: mirror(Settings.Autonomous.BlueClose.PRESET_1_PREP),
						isBlue ? Settings.Autonomous.BlueClose.PRESET_1_END
								: mirror(Settings.Autonomous.BlueClose.PRESET_1_END));
			
			case CLOSE_LAUNCH_1:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.PRESET_1_END
								: mirror(Settings.Autonomous.BlueClose.PRESET_1_END),
						isBlue ? Settings.Autonomous.BlueClose.LAUNCH_1
								: mirror(Settings.Autonomous.BlueClose.LAUNCH_1));
			
			case CLOSE_PRESET_2_PREP:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.LAUNCH_1
								: mirror(Settings.Autonomous.BlueClose.LAUNCH_1),
						isBlue ? Settings.Autonomous.BlueClose.PRESET_2_PREP
								: mirror(Settings.Autonomous.BlueClose.PRESET_2_PREP));
			
			case CLOSE_PRESET_2_END:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.PRESET_2_PREP
								: mirror(Settings.Autonomous.BlueClose.PRESET_2_PREP),
						isBlue ? Settings.Autonomous.BlueClose.PRESET_2_END
								: mirror(Settings.Autonomous.BlueClose.PRESET_2_END));
			
			case CLOSE_LAUNCH_2:
				return buildCurvedPath(
						isBlue ? Settings.Autonomous.BlueClose.PRESET_2_END
								: mirror(Settings.Autonomous.BlueClose.PRESET_2_END),
						isBlue ? Settings.Autonomous.BlueClose.LAUNCH_2
								: mirror(Settings.Autonomous.BlueClose.LAUNCH_2));
			
			case CLOSE_PRESET_3_PREP:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.LAUNCH_2
								: mirror(Settings.Autonomous.BlueClose.LAUNCH_2),
						isBlue ? Settings.Autonomous.BlueClose.PRESET_3_PREP
								: mirror(Settings.Autonomous.BlueClose.PRESET_3_PREP));
			
			case CLOSE_PRESET_3_END:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.PRESET_3_PREP
								: mirror(Settings.Autonomous.BlueClose.PRESET_3_PREP),
						isBlue ? Settings.Autonomous.BlueClose.PRESET_3_END
								: mirror(Settings.Autonomous.BlueClose.PRESET_3_END));
			
			case CLOSE_LAUNCH_3:
				return buildCurvedPath(
						isBlue ? Settings.Autonomous.BlueClose.PRESET_3_END
								: mirror(Settings.Autonomous.BlueClose.PRESET_3_END),
						isBlue ? Settings.Autonomous.BlueClose.LAUNCH_3
								: mirror(Settings.Autonomous.BlueClose.LAUNCH_3));
			
			case CLOSE_PARK:
				return buildLinearPath(
						isBlue ? Settings.Autonomous.BlueClose.LAUNCH_3
								: mirror(Settings.Autonomous.BlueClose.LAUNCH_3),
						isBlue ? Settings.Autonomous.BlueClose.PARK : mirror(Settings.Autonomous.BlueClose.PARK));
			
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
	 * Mirrors a pose across the field centerline for red alliance.
	 * Field width is 144 inches (standard FTC field).
	 * Takes a BLUE pose and returns the mirrored RED pose.
	 */
	private Pose mirror(Pose bluePose) {
		return new Pose(
				Settings.Field.WIDTH - bluePose.getX(), // Mirror X coordinate
				bluePose.getY(), // Y stays the same
				Math.PI - bluePose.getHeading() // Mirror heading
		);
	}
	
	/**
	 * Mirrors a BezierCurve for red alliance.
	 * Takes a BLUE curve and returns the mirrored RED curve.
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
