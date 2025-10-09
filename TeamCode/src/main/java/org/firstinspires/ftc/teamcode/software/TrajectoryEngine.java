package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;

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
		double deltaX = goalPose.getX() - robotPose.getX();
		double deltaY = goalPose.getY() - robotPose.getY();
		double horizontalDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
		
		// Calculate launcher height (accounting for current pitch rotation)
		double pitchRadians = Math.toRadians(currentPitchDegrees);
		double cosTheta = Math.cos(pitchRadians);
		double sinTheta = Math.sin(pitchRadians);
		
		double launcherRelativeX = Settings.Aiming.LAUNCHER_FROM_PITCH_AXIS_BACK_INCHES;
		double launcherRelativeY = Settings.Aiming.LAUNCHER_FROM_PITCH_AXIS_UP_INCHES;
		
		double rotatedLauncherY = launcherRelativeX * sinTheta + launcherRelativeY * cosTheta;
		double actualLauncherHeight = Settings.Aiming.PITCH_AXIS_HEIGHT_INCHES + rotatedLauncherY;
		
		// Calculate target height (AprilTag + offset)
		double targetHeight = Settings.Aiming.APRILTAG_CENTER_HEIGHT_INCHES
				+ Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES;
		
		// Calculate height difference
		double heightDifference = targetHeight - actualLauncherHeight;
		
		// Calculate launch angle using simple trigonometry
		double launchAngleRadians = Math.atan2(heightDifference, horizontalDistance);
		double launchAngleDegrees = Math.toDegrees(launchAngleRadians);
		
		// Simple aiming uses default wheel speed and no yaw correction
		double launchSpeed = Settings.Aiming.wheelRpmToVelocity(Settings.Aiming.WHEEL_SPEED_RPM);
		
		return new AimingSolution(0.0, launchAngleDegrees - currentPitchDegrees, launchSpeed, true);
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
		LLResult limelightResult = limelightManager.detectGoal();
		
		if (limelightResult.getFiducialResults().isEmpty()) {
			return AimingSolution.invalid();
		}
		
		int targetId = (allianceColor == MatchSettings.AllianceColor.BLUE ? 20 : 24);
		
		for (LLResultTypes.FiducialResult fiducial : limelightResult.getFiducialResults()) {
			if (fiducial.getFiducialId() == targetId) {
				// Horizontal: yaw offset from camera center (tx)
				double yawOffset = fiducial.getTargetXDegrees();
				
				// Get vertical angle from limelight to target (ty)
				// This is relative to the limelight's current orientation
				double tyDegrees = fiducial.getTargetYDegrees();
				
				// Prevent numerical issues with very small angles
				if (Math.abs(tyDegrees) < 0.5) {
					tyDegrees = Math.signum(tyDegrees) * 0.5;
				}
				
				// Step 1: Calculate absolute angle to target from horizontal
				// The limelight sees an angle relative to its orientation
				// The absolute angle is the launcher pitch + the limelight's reading
				double absoluteAngleDegrees = currentPitchDegrees + tyDegrees;
				double absoluteAngleRadians = Math.toRadians(absoluteAngleDegrees);
				
				// Step 2: Calculate current limelight position after rotation
				// The pitch axis is at a fixed position
				double pitchAxisX = Settings.Aiming.PITCH_AXIS_FORWARD_OFFSET_INCHES;
				double pitchAxisY = Settings.Aiming.PITCH_AXIS_HEIGHT_INCHES;
				
				// Limelight position relative to pitch axis (before rotation)
				double limelightRelativeX = Settings.Aiming.LIMELIGHT_FROM_PITCH_AXIS_FORWARD_INCHES;
				double limelightRelativeY = -Settings.Aiming.LIMELIGHT_FROM_PITCH_AXIS_DOWN_INCHES; // negative because
				// "down"
				
				// Apply rotation matrix around pitch axis
				double pitchRadians = Math.toRadians(currentPitchDegrees);
				double cosTheta = Math.cos(pitchRadians);
				double sinTheta = Math.sin(pitchRadians);
				
				double rotatedLimelightX = limelightRelativeX * cosTheta - limelightRelativeY * sinTheta;
				double rotatedLimelightY = limelightRelativeX * sinTheta + limelightRelativeY * cosTheta;
				
				// Actual limelight position in field coordinates
				double actualLimelightX = pitchAxisX + rotatedLimelightX;
				double actualLimelightY = pitchAxisY + rotatedLimelightY;
				
				// Step 3: Calculate target position using absolute angle
				// From limelight position, shoot a ray at the absolute angle until it hits the
				// AprilTag height
				double heightFromLimelightToTag = Settings.Aiming.APRILTAG_CENTER_HEIGHT_INCHES - actualLimelightY;
				double horizontalDistanceFromLimelight = heightFromLimelightToTag / Math.tan(absoluteAngleRadians);
				
				// Ensure valid distance
				if (horizontalDistanceFromLimelight <= 0 || horizontalDistanceFromLimelight > 300) {
					continue; // Invalid geometry
				}
				
				// Target X position
				double targetX = actualLimelightX + horizontalDistanceFromLimelight;
				
				// Step 4: Calculate current launcher position after rotation
				double launcherRelativeX = Settings.Aiming.LAUNCHER_FROM_PITCH_AXIS_BACK_INCHES;
				double launcherRelativeY = Settings.Aiming.LAUNCHER_FROM_PITCH_AXIS_UP_INCHES;
				
				double rotatedLauncherX = launcherRelativeX * cosTheta - launcherRelativeY * sinTheta;
				double rotatedLauncherY = launcherRelativeX * sinTheta + launcherRelativeY * cosTheta;
				
				double actualLauncherX = pitchAxisX + rotatedLauncherX;
				double actualLauncherY = pitchAxisY + rotatedLauncherY;
				
				// Step 5: Calculate distance from launcher to target
				double horizontalDistanceFromLauncher = targetX - actualLauncherX;
				
				if (horizontalDistanceFromLauncher <= 0) {
					continue; // Target is behind launcher
				}
				
				// Step 6: Calculate desired aim point height (AprilTag + offset)
				double targetHeight = Settings.Aiming.APRILTAG_CENTER_HEIGHT_INCHES
						+ Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES;
				double heightFromLauncherToTarget = targetHeight - actualLauncherY;
				
				// Step 7: Calculate required launch angle from launcher position
				double launchAngleRadians = Math.atan2(heightFromLauncherToTarget, horizontalDistanceFromLauncher);
				double launchAngleDegrees = Math.toDegrees(launchAngleRadians);
				
				// Complex aiming uses default wheel speed (can be upgraded to physics-based
				// calculation)
				double launchSpeed = Settings.Aiming.wheelRpmToVelocity(Settings.Aiming.WHEEL_SPEED_RPM);
				
				return new AimingSolution(yawOffset, launchAngleDegrees, launchSpeed, true);
			}
		}
		
		return AimingSolution.invalid();
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
