package org.firstinspires.ftc.teamcode.software;

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
	
	/**
	 * Solves for the launch angle(s) required to hit a target.
	 *
	 * @param V0 The initial speed of the projectile (m/s).
	 * @param r  The horizontal distance to the target (m).
	 * @param h  The vertical height of the target (m).
	 * @return A List of possible launch angles in radians. The list will contain:
	 * - 2 solutions (high and low arc)
	 * - 1 solution (minimum energy launch)
	 * - 0 solutions (target is unreachable with the given V0)
	 */
	public List<Double> solveForTheta(double V0, double r, double h) {
		final double g = 386.2; // in/s/s
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
		
		// Simple aiming uses default wheel speed and no yaw correction
		double launchSpeed = Settings.Aiming.wheelRpmToTangentialWheelVelocity(Settings.Aiming.WHEEL_SPEED_RPM);
		
		return new AimingSolution(getYawOffset(allianceColor), launchAngleDegrees - currentPitchDegrees, launchSpeed, true);
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
		if (Double.isNaN(yawOffset)) {
			return AimingSolution.invalid();
		}
		
		double radiusFromGoal = getDistanceFromGoal(drivetrain.getPose(),
				(allianceColor == MatchSettings.AllianceColor.BLUE)
						? Settings.Field.BLUE_GOAL_POSE
						: Settings.Field.RED_GOAL_POSE);
		
		// find the yaw offset)
		Double bestAngle = Double.NaN;
		double bestRPM = Double.NaN;
		double maxRatio = Double.NEGATIVE_INFINITY;
		
		
		// for each relatively possible launcher RPM
		for (double rpm = Settings.Aiming.MIN_WHEEL_SPEED_RPM;
		     rpm <= Settings.Aiming.MAX_WHEEL_SPEED_RPM;
		     rpm += 100) {
			double initialBallSpeed = initialBallSpeedFromRPM(rpm);
			double currentLauncherHeight = calculateLauncherHeight(currentPitchDegrees);
			List<Double> launchAngles = solveForTheta(initialBallSpeed, radiusFromGoal, Settings.Field.BLUE_GOAL_AIM_3D[2] - currentLauncherHeight);
			for (Double theta : launchAngles) {
				double finalVelocityX = initialBallSpeed * Math.cos(theta);
				double t = radiusFromGoal / finalVelocityX;
				double finalVelocityY = initialBallSpeed * Math.sin(theta) - 386.2 * t;
				
				// calculate the ratio
				double currentRatio = Math.abs(finalVelocityY / finalVelocityX);
				
				if (currentRatio > maxRatio) {
					maxRatio = currentRatio;
					bestAngle = Math.toDegrees(theta);
					bestRPM = rpm;
				}
				
			}
			
		}
		
		if (Double.isNaN(bestAngle) || Double.isNaN(bestRPM)) {
			return AimingSolution.invalid();
		}
		
		return new AimingSolution(yawOffset, bestAngle - currentPitchDegrees, Settings.Aiming.wheelRpmToTangentialWheelVelocity(bestRPM), true);
	}
	
	private double getDistanceFromGoal(Pose robotPose, Pose goalPose) {
		double deltaX = goalPose.getX() - robotPose.getX();
		double deltaY = goalPose.getY() - robotPose.getY();
		
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}
	
	private double initialBallSpeedFromRPM(double rpm) {
		// mass of wheel times radius of wheel squared times rpm of motor, quantity divided by mass of ball, times coefficient we linearize for
		double numerator = Settings.Launcher.WHEEL_MASS_KG
				* Math.pow(Settings.Aiming.WHEEL_DIAMETER_INCHES / 2, 2)
				* Math.pow(rpm, 2);
		double denominator = Settings.Field.BALL_MASS_KG;
		
		return Settings.Launcher.LAUNCHER_SHOT_EFFICIENCY_COEFFICIENT * (numerator / denominator);
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
		public final double launchVelocityInchesPerSec; // Required launch velocity
		
		/**
		 * Constructor for a valid aiming solution.
		 */
		public AimingSolution(double horizontalOffset, double verticalOffset,
		                      double launchVelocity, boolean hasTarget) {
			this.hasTarget = hasTarget;
			this.horizontalOffsetDegrees = horizontalOffset;
			this.verticalOffsetDegrees = verticalOffset;
			this.launchVelocityInchesPerSec = launchVelocity;
		}
		
		/**
		 * Constructor for an invalid solution, used when no target is found.
		 */
		public static AimingSolution invalid() {
			return new AimingSolution(Double.NaN, Double.NaN, Double.NaN, false);
		}
		
		/**
		 * Gets the required wheel speed in RPM for this solution.
		 */
		public double getRequiredWheelSpeedRPM() {
			return Settings.Aiming.velocityToWheelRpm(launchVelocityInchesPerSec);
		}
	}
}
