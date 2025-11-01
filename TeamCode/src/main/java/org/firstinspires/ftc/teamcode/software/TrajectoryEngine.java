package org.firstinspires.ftc.teamcode.software;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.Drivetrain.Position;

/**
 * Simplified Trajectory Engine that uses preset launch angles based on
 * which shooting position (CLOSE_SHOOT or FAR_SHOOT) is closer.
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
	 * Determines which shooting position is closer and returns the preset aiming
	 * solution.
	 *
	 * @param allianceColor The alliance color to determine target ID for yaw
	 *                      correction
	 * @return An {@link AimingSolution} with preset angles and RPM based on closest
	 * position
	 */
	public AimingSolution getAimingOffsets(MatchSettings.AllianceColor allianceColor, double currentPitchDegrees) {
		Pose currentPose = drivetrain.getPose();
		Pose closeShootPose = drivetrain.getPositionPose(Position.CLOSE_SHOOT);
		Pose farShootPose = drivetrain.getPositionPose(Position.FAR_SHOOT);
		
		// Determine which position is closer
		double distanceToClose = getDistance(currentPose, closeShootPose);
		double distanceToFar = getDistance(currentPose, farShootPose);
		
		Position closestPosition = (distanceToClose < distanceToFar) ? Position.CLOSE_SHOOT : Position.FAR_SHOOT;
		
		// Get preset values for the closest position
		double presetPitch;
		double presetRPM;
		
		if (closestPosition == Position.CLOSE_SHOOT) {
			presetPitch = Settings.Aiming.CLOSE_SHOOT_PITCH_DEGREES;
			presetRPM = Settings.Aiming.CLOSE_SHOOT_RPM;
		} else {
			presetPitch = Settings.Aiming.FAR_SHOOT_PITCH_DEGREES;
			presetRPM = Settings.Aiming.FAR_SHOOT_RPM;
		}
		
		// Get yaw offset from limelight if enabled
		double yawOffset = getYawOffset(allianceColor);
		// If yaw correction is required but we can't get it, return invalid
		if (Settings.Launcher.CORRECT_YAW && Double.isNaN(yawOffset)) {
			return AimingSolution.invalid();
		}
		
		// Return solution with preset pitch and calculated yaw offset
		return new AimingSolution(
				yawOffset,
				presetPitch,
				presetRPM,
				true);
	}
	
	/**
	 * Calculates the distance between two poses.
	 */
	private double getDistance(Pose pose1, Pose pose2) {
		double deltaX = pose2.getX() - pose1.getX();
		double deltaY = pose2.getY() - pose1.getY();
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}
	
	/**
	 * Gets the yaw offset from the camera center.
	 * Safely handles the Nullable limelight based on settings.
	 *
	 * @param allianceColor The alliance color to determine target ID
	 * @return The yaw offset from the camera center in degrees
	 */
	private Double getYawOffset(MatchSettings.AllianceColor allianceColor) {
		if (!Settings.Launcher.CORRECT_YAW) {
			return 0.0;
		}
		if (limelightManager == null) {
			return Double.NaN;
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
	 * - rpm: Required launch motor RPM
	 */
	public static class AimingSolution {
		public final boolean hasTarget;
		public final double horizontalOffsetDegrees; // Yaw offset from center
		public final double verticalOffsetDegrees; // Absolute pitch angle from horizontal
		public final double rpm; // Required launch motor rpm
		
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
}