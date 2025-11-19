package org.firstinspires.ftc.teamcode.software;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain.Position;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Simplified Trajectory Engine that uses preset launch angles based on
 * which shooting position (CLOSE_SHOOT or FAR_SHOOT) is closer.
 */
public class TrajectoryEngine {
	MechanismManager mechanisms;
	
	public TrajectoryEngine(MechanismManager mechanisms) {
		this.mechanisms = mechanisms;
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
		Pose currentPose = mechanisms.drivetrain.getPose();
		Pose closeShootPose = mechanisms.drivetrain.getPositionPose(Position.CLOSE_SHOOT);
		Pose farShootPose = mechanisms.drivetrain.getPositionPose(Position.FAR_SHOOT);
		
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
		
		
		// Return solution with preset pitch and calculated yaw offset
		return new AimingSolution(
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
		public final double verticalOffsetDegrees; // Absolute pitch angle from horizontal
		public final double rpm; // Required launch motor rpm
		
		/**
		 * Constructor for a valid aiming solution.
		 */
		public AimingSolution(double verticalOffset,
		                      double rpm, boolean hasTarget) {
			this.hasTarget = hasTarget;
			this.verticalOffsetDegrees = verticalOffset;
			this.rpm = rpm;
		}
		
		/**
		 * Constructor for an invalid solution, used when no target is found.
		 */
		public static AimingSolution invalid() {
			return new AimingSolution(Double.NaN, Double.NaN, false);
		}
	}
}