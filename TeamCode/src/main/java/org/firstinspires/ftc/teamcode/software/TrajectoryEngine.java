package org.firstinspires.ftc.teamcode.software;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Aiming.GRAVITY_INCHES_PER_SEC_SQ;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;

import java.util.ArrayList;
import java.util.List;

/**
 * The Trajectory Engine uses a launcher-mounted Limelight to calculate optimal
 * aiming solutions.
 * Supports both simple offset-based aiming and complex physics-based projectile
 * calculations.
 */
public class TrajectoryEngine {
	
	public static final MonotoneHermite curve = new MonotoneHermite(Settings.Launcher.OUTTAKE_DATA_X, Settings.Launcher.OUTTAKE_DATA_Y);
	private final LimelightManager limelightManager;
	private final MatchSettings matchSettings;
	private final Drivetrain drivetrain;
	
	public TrajectoryEngine(LimelightManager limelightManager, MatchSettings matchSettings, Drivetrain drivetrain) {
		this.limelightManager = limelightManager;
		this.matchSettings = matchSettings;
		this.drivetrain = drivetrain;
	}
	
	/**
	 * Calculates the absolute height of the launcher's outtake point from the field.
	 * This function accounts for the launcher's rotation around its pitch axis.
	 *
	 * @param currentPitchDegrees The current pitch angle of the launcher in degrees.
	 * @return The calculated height of the launcher outtake in inches from the field floor.
	 */
	public static double calculateLauncherHeight(double currentPitchDegrees) {
		// Convert the current pitch to radians for trigonometric functions
		double pitchRadians = Math.toRadians(currentPitchDegrees);
		
		// Calculate the launcher's vertical offset from the pitch axis after rotation.
		// This uses a 2D rotation matrix to find the new Y-coordinate.
		double rotatedLauncherY =
				Settings.Aiming.LAUNCHER_FROM_PITCH_AXIS_BACK_INCHES * Math.sin(pitchRadians) +
						Settings.Aiming.LAUNCHER_FROM_PITCH_AXIS_UP_INCHES * Math.cos(pitchRadians);
		
		// Add the base height of the pitch axis to get the absolute height from the field
		return Settings.Aiming.PITCH_AXIS_HEIGHT_INCHES + rotatedLauncherY;
	}
	
	public double launcherExitSpeedInchesPerSecond(double rpm) {
		// return LAUNCH_EFFICIENCY * Math.PI * Settings.Aiming.WHEEL_DIAMETER_INCHES * rpm / 60; // 60 sec/min
		return curve.evaluate(rpm / 7250); // approximate using a MonotoneHermite curve. curve undershoots so 7500 instead of 6000
	}
	
	/**
	 * Solves for the launch angle(s) required to hit a target.
	 *
	 * @param V0 The initial speed of the projectile (in/s).
	 * @param r  The horizontal distance to the target (in).
	 * @param h  The vertical height of the target (in).
	 * @return A List of possible launch angles in radians. The list will contain:
	 * - 2 solutions (high and low arc)
	 * - 1 solution (minimum energy launch)
	 * - 0 solutions (target is unreachable with the given V0)
	 */
	public List<Double> solveForTheta(double V0, double r, double h) {
		final double g = GRAVITY_INCHES_PER_SEC_SQ; // in/s/s
		List<Double> angles = new ArrayList<>();
		
		// Define the coefficients for the quadratic equation Au^2 + Bu + C = 0,
		// where u = tan(theta).
		double A = (g * r * r) / (2 * V0 * V0);
		double B = -r;
		double C = h + A;
		
		// Calculate the discriminant
		double discriminant = B * B - 4 * A * C;
		
		// Check if there are real solutions
		if (discriminant < 0) {
			// No real solutions, the target is out of range.
			return angles; // Return empty list
		}
		
		double sqrtDiscriminant = Math.sqrt(discriminant);
		
		// Calculate the two possible values for u = tan(theta)
		double u1 = (-B + sqrtDiscriminant) / (2 * A);
		double u2 = (-B - sqrtDiscriminant) / (2 * A);
		
		// Calculate the angles from the tangents
		double theta1 = Math.atan(u1);
		double theta2 = Math.atan(u2);
		
		angles.add(theta1);
		// If the discriminant is non-zero, the second solution is distinct
		if (discriminant > 1e-9) { // Using a small epsilon to handle floating point errors
			angles.add(theta2);
		}
		
		return angles;
	}
	
	/**
	 * Gets the current aiming solution from the Limelight camera.
	 * Routes to either simple or complex aiming based on settings.
	 *
	 * @param allianceColor       The alliance color to determine target ID
	 * @param currentPitchDegrees The current pitch angle of the launcher in degrees
	 * @return An {@link AimingSolution} object containing the targeting data.
	 */
	public AimingSolution getAimingOffsets(MatchSettings.AllianceColor allianceColor, double currentPitchDegrees) {
		if (Settings.Aiming.USE_COMPLEX_AIMING) {
			return aimComplex(allianceColor, currentPitchDegrees);
		} else {
			return aimSimple(allianceColor, currentPitchDegrees);
		}
	}
	
	/**
	 * Simple aiming: uses robot pose and goal pose to calculate launch angle.
	 * Much simpler than complex aiming - just uses geometry based on known
	 * positions.
	 *
	 * @param allianceColor       The alliance color to determine target goal
	 * @param currentPitchDegrees The current pitch angle of the launcher
	 * @return An {@link AimingSolution} with:
	 * - horizontal: 0 (no yaw correction - assumes robot is positioned
	 * correctly)
	 * - vertical: calculated launch angle from robot position to goal
	 * - velocity: default wheel speed
	 */
	private AimingSolution aimSimple(MatchSettings.AllianceColor allianceColor, double currentPitchDegrees) {
		// Get robot's current position
		com.pedropathing.geometry.Pose robotPose = drivetrain.getPose();
		
		// Get target goal pose based on alliance
		com.pedropathing.geometry.Pose goalPose = (allianceColor == MatchSettings.AllianceColor.BLUE)
				? Settings.Field.BLUE_GOAL_POSE
				: Settings.Field.RED_GOAL_POSE;
		
		// Calculate horizontal distance from robot to goal
		double radiusFromGoal = getDistanceFromGoal(drivetrain.getPose(),
				(allianceColor == MatchSettings.AllianceColor.BLUE)
						? Settings.Field.BLUE_GOAL_POSE
						: Settings.Field.RED_GOAL_POSE);
		
		// Calculate launcher height (accounting for current pitch rotation)
		
		// Calculate launcher height (accounting for current pitch rotation)
		double actualLauncherHeight = calculateLauncherHeight(currentPitchDegrees);
		
		// Calculate target height (AprilTag)
		// Calculate target height (AprilTag + offset)
		double targetHeight = Settings.Aiming.APRILTAG_CENTER_HEIGHT_INCHES
				+ Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES;
		
		// Calculate height difference
		double heightDifference = targetHeight - actualLauncherHeight;
		
		// Calculate launch angle using simple trigonometry
		double launchAngleRadians = Math.atan2(heightDifference, radiusFromGoal);
		double launchAngleDegrees = Math.toDegrees(launchAngleRadians);
		
		return new AimingSolution(getYawOffset(allianceColor), launchAngleDegrees - currentPitchDegrees, Settings.Aiming.DEFAULT_WHEEL_SPEED_RPM, true);
	}
	
	/**
	 * Complex aiming: accounts for launcher rotation and calculates the required
	 * launch angle.
	 * The limelight is mounted on the launcher and rotates with it around the pitch
	 * axis.
	 * <p>
	 * Key steps:
	 * 1. Get limelight's angle reading (ty) - this is relative to the limelight's
	 * orientation
	 * 2. Calculate the absolute angle to the target (launcher pitch + ty)
	 * 3. Calculate limelight's current position after rotation around pitch axis
	 * 4. Use absolute angle and limelight position to find target position
	 * 5. Calculate launcher's current position after rotation around pitch axis
	 * 6. Calculate required launch angle from launcher to target
	 *
	 * @param allianceColor       The alliance color to determine target ID
	 * @param currentPitchDegrees The current pitch angle of the launcher
	 * @return An {@link AimingSolution} with:
	 * - horizontal: yaw offset from camera center (degrees, -10 to +10)
	 * - vertical: absolute launch angle from horizontal (degrees, 0 to 90)
	 * - velocity: default wheel speed
	 */
	private AimingSolution aimComplex(MatchSettings.AllianceColor allianceColor, double currentPitchDegrees) {
		double yawOffset = getYawOffset(allianceColor);
		if (Double.isNaN(yawOffset)) return AimingSolution.invalid();
		
		double radiusFromGoal = getDistanceFromGoal(drivetrain.getPose(),
				(allianceColor == MatchSettings.AllianceColor.BLUE)
						? Settings.Field.BLUE_GOAL_POSE
						: Settings.Field.RED_GOAL_POSE);
		
		double bestAngle = Double.NaN;
		double bestRPM = Double.NaN;
		double minRPM = Double.POSITIVE_INFINITY;
		
		for (double rpm = Settings.Aiming.MIN_WHEEL_SPEED_RPM;
		     rpm <= Settings.Aiming.MAX_WHEEL_SPEED_RPM;
		     rpm += Settings.Aiming.WHEEL_SPEED_OPTIMIZATION_STEP_RPM) {
			
			double initialBallSpeed = launcherExitSpeedInchesPerSecond(rpm);
			double currentLauncherHeight = calculateLauncherHeight(currentPitchDegrees);
			
			List<Double> launchAngles = solveForTheta(initialBallSpeed,
					radiusFromGoal,
					Settings.Field.BLUE_GOAL_AIM_3D[2] - currentLauncherHeight);
			
			for (Double theta : launchAngles) {
				double vx = initialBallSpeed * Math.cos(theta);
				double t = radiusFromGoal / vx;
				double vyFinal = initialBallSpeed * Math.sin(theta) - GRAVITY_INCHES_PER_SEC_SQ * t;
				double entryAngle = Math.toDegrees(Math.abs(Math.atan2(vyFinal, vx)));
				
				// constraint one: ball must enter goal from above
				// contraint two: motors must have a reasonable angle for balls to enter it
				if (entryAngle >= Settings.Aiming.MIN_ENTRY_ANGLE_DEGREES && rpm < minRPM &&
						bestAngle > Settings.Aiming.MAX_LAUNCHER_ANGLE_DEGREES_FROM_HORIZONTAL
				) {
					minRPM = rpm;
					bestAngle = Math.toDegrees(theta);
					bestRPM = rpm;
				}
			}
		}
		
		if (Double.isNaN(bestAngle) || Double.isNaN(bestRPM)) return AimingSolution.invalid();
		
		return new AimingSolution(
				yawOffset,
				bestAngle - currentPitchDegrees,
				bestRPM,
				true
		);
	}
	
	
	private double getDistanceFromGoal(Pose robotPose, Pose goalPose) {
		double deltaX = goalPose.getX() - robotPose.getX();
		double deltaY = goalPose.getY() - robotPose.getY();
		
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}
	
	private Double getYawOffset(MatchSettings.AllianceColor allianceColor) {
		if (!Settings.Launcher.CORRECT_YAW) {
			return 0.0;
		}
		LLResult limelightResult = limelightManager.detectGoal();
		
		if (limelightResult.getFiducialResults().isEmpty()) {
			return Double.NaN;
		}
		
		int targetId = (allianceColor == MatchSettings.AllianceColor.BLUE ? 20 : 24);
		
		for (LLResultTypes.FiducialResult fiducial : limelightResult.getFiducialResults()) {
			if (fiducial.getFiducialId() == targetId) {
				return fiducial.getTargetXDegrees();
			}
		}
		return Double.NaN;
	}
	
	
	/**
	 * Data class to hold the complete aiming solution including angles and launch
	 * velocity.
	 * <p>
	 * Coordinate System:
	 * - horizontalOffsetDegrees: Yaw offset from camera center (-10° to +10°)
	 * * 0° = perfectly centered on target
	 * * Positive = target is to the right, need to rotate right
	 * * Negative = target is to the left, need to rotate left
	 * <p>
	 * - verticalOffsetDegrees: Absolute launch angle from horizontal (0° to 90°)
	 * * 0° = horizontal/parallel to ground
	 * * 45° = 45° launch angle
	 * * 90° = straight up
	 * <p>
	 * - launchVelocityInchesPerSec: Required launch velocity for the shot
	 */
	public static class AimingSolution {
		public final boolean hasTarget;
		public final double horizontalOffsetDegrees; // Yaw offset from center
		public final double verticalOffsetDegrees; // Absolute launch angle
		public final double rpm; // Required launch velocity
		
		/**
		 * Constructor for a valid aiming solution.
		 */
		public AimingSolution(double horizontalOffset, double verticalOffset,
		                      double rpm, boolean hasTarget) {
			this.hasTarget = hasTarget;
			this.horizontalOffsetDegrees = horizontalOffset;
			this.verticalOffsetDegrees = verticalOffset;
			this.rpm = rpm;
		}
		
		/**
		 * Constructor for an invalid solution, used when no target is found.
		 */
		public static AimingSolution invalid() {
			return new AimingSolution(Double.NaN, Double.NaN, Double.NaN, false);
		}
	}
	
	
	public static class MonotoneHermite {
		private final double[] xs;
		private final double[] ys;
		private final double[] m; // slopes at knots
		
		public MonotoneHermite(double[] xs, double[] ys) {
			if (xs.length != ys.length) throw new IllegalArgumentException("lengths differ");
			if (xs.length < 2) throw new IllegalArgumentException("need >=2 points");
			this.xs = xs.clone();
			this.ys = ys.clone();
			this.m = computeSlopes(xs, ys);
		}
		
		private double[] computeSlopes(double[] x, double[] y) {
			int n = x.length;
			double[] delta = new double[n - 1];
			for (int i = 0; i < n - 1; i++) delta[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
			
			double[] m = new double[n];
			// Endpoints: use one-sided slopes
			m[0] = delta[0];
			m[n - 1] = delta[n - 2];
			// Interior initial slopes: average of adjacent secants
			for (int i = 1; i < n - 1; i++) m[i] = (delta[i - 1] + delta[i]) * 0.5;
			
			// Enforce monotonicity (Fritsch-Carlson)
			for (int i = 0; i < n - 1; i++) {
				if (delta[i] == 0.0) {
					m[i] = 0.0;
					m[i + 1] = 0.0;
				} else {
					double a = m[i] / delta[i];
					double b = m[i + 1] / delta[i];
					double s = a * a + b * b;
					if (s > 9.0) {
						double tau = 3.0 / Math.sqrt(s);
						m[i] = tau * a * delta[i];
						m[i + 1] = tau * b * delta[i];
					}
				}
			}
			return m;
		}
		
		public double evaluate(double x) {
			// clamp
			if (x <= xs[0]) return ys[0];
			if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
			
			// find interval
			int i = 0;
			int j = xs.length - 1;
			while (j - i > 1) {
				int mid = (i + j) >>> 1;
				if (xs[mid] <= x) i = mid;
				else j = mid;
			}
			
			double x0 = xs[i], x1 = xs[i + 1];
			double y0 = ys[i], y1 = ys[i + 1];
			double m0 = m[i], m1 = m[i + 1];
			double h = x1 - x0;
			double t = (x - x0) / h;
			
			double t2 = t * t;
			double t3 = t2 * t;
			double h00 = 2 * t3 - 3 * t2 + 1;
			double h10 = t3 - 2 * t2 + t;
			double h01 = -2 * t3 + 3 * t2;
			double h11 = t3 - t2;
			
			return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
		}
	}
}