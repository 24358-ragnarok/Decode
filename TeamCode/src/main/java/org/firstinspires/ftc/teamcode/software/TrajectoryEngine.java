package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
	
	public TrajectoryEngine(LimelightManager limelightManager, MatchSettings matchSettings) {
		this.limelightManager = limelightManager;
		this.matchSettings = matchSettings;
	}
	
	/**
	 * Gets the current aiming solution from the Limelight camera.
	 * Routes to either simple or complex aiming based on settings.
	 *
	 * @return An {@link AimingSolution} object containing the targeting data.
	 */
	public AimingSolution getAimingOffsets(MatchSettings.AllianceColor allianceColor) {
		if (Settings.Aiming.USE_COMPLEX_AIMING) {
			return aimComplex(allianceColor);
		} else {
			return aimSimple(allianceColor);
		}
	}
	
	/**
	 * Simple aiming: uses direct angle offsets from Limelight with a fixed vertical
	 * offset.
	 * This method aims at a point offset from the AprilTag without physics
	 * calculations.
	 * <p>
	 * Uses trigonometry with the vertical angle (ty) instead of 3D pose data.
	 *
	 * @param allianceColor The alliance color to determine target ID
	 * @return An {@link AimingSolution} with:
	 * - horizontal: yaw offset from camera center (degrees, -10 to +10)
	 * - vertical: absolute launch angle from horizontal (degrees, 0 to 90)
	 * - velocity: default wheel speed
	 */
	private AimingSolution aimSimple(MatchSettings.AllianceColor allianceColor) {
		LLResult limelightResult = limelightManager.detectGoal();
		
		if (limelightResult.getFiducialResults().isEmpty()) {
			return AimingSolution.invalid();
		}
		
		int targetId = (allianceColor == MatchSettings.AllianceColor.BLUE ? 20 : 24);
		
		for (LLResultTypes.FiducialResult fiducial : limelightResult.getFiducialResults()) {
			if (fiducial.getFiducialId() == targetId) {
				// Horizontal: yaw offset from camera center (tx)
				// This is the error that needs to be corrected
				double yawOffset = fiducial.getTargetXDegrees();
				
				// Get vertical angle from camera to target (ty)
				// Positive = target is above camera horizontal
				// Negative = target is below camera horizontal
				double verticalAngleDegrees = fiducial.getTargetYDegrees();
				
				// Skip if angle is too small (would cause numerical issues)
				if (Math.abs(verticalAngleDegrees) < 0.5) {
					// Target is essentially at camera level, use a minimum angle
					verticalAngleDegrees = 0.5;
				}
				
				double verticalAngleRadians = Math.toRadians(verticalAngleDegrees);
				
				// Calculate height difference from camera/launcher to AprilTag
				// This assumes camera and launcher are at same height (or close enough)
				double heightDiffToTagInches = Settings.Aiming.GOAL_HEIGHT_INCHES
						- Settings.Aiming.LAUNCHER_HEIGHT_INCHES;
				
				// Use trigonometry to find horizontal distance:
				// tan(ty) = heightDiff / distance
				// distance = heightDiff / tan(ty)
				double horizontalDistanceInches = heightDiffToTagInches / Math.tan(verticalAngleRadians);
				
				// Ensure we have a valid distance (positive and reasonable)
				if (horizontalDistanceInches <= 0 || horizontalDistanceInches > 300) {
					// Invalid geometry, skip this solution
					continue;
				}
				
				// Calculate desired aim point: AprilTag + vertical offset
				double desiredHeightInches = Settings.Aiming.GOAL_HEIGHT_INCHES
						+ Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES;
				
				// Calculate height difference from launcher to desired aim point
				double heightDiffToAimPointInches = desiredHeightInches
						- Settings.Aiming.LAUNCHER_HEIGHT_INCHES;
				
				// Calculate the launch angle to reach the aim point
				// atan2(height, distance) gives us the angle from horizontal
				double launchAngleRadians = Math.atan2(heightDiffToAimPointInches, horizontalDistanceInches);
				double launchAngleDegrees = Math.toDegrees(launchAngleRadians);
				
				// Simple aiming uses default wheel speed
				double launchSpeed = Settings.Aiming.wheelRpmToVelocity(Settings.Aiming.WHEEL_SPEED_RPM);
				
				return new AimingSolution(yawOffset, launchAngleDegrees, launchSpeed, true);
			}
		}
		
		return AimingSolution.invalid();
	}
	
	/**
	 * Complex aiming: uses projectile motion physics to calculate optimal launch
	 * angle and speed.
	 * Considers gravity, distance, height difference, and minimizes impact
	 * velocity.
	 *
	 * @param allianceColor The alliance color to determine target ID
	 * @return An {@link AimingSolution} with:
	 * - horizontal: yaw offset from camera center (degrees, -10 to +10)
	 * - vertical: optimal launch angle from horizontal (degrees, 0 to 90)
	 * - velocity: calculated optimal launch velocity
	 */
	private AimingSolution aimComplex(MatchSettings.AllianceColor allianceColor) {
		LLResult limelightResult = limelightManager.detectGoal();
		
		if (limelightResult.getFiducialResults().isEmpty()) {
			return AimingSolution.invalid();
		}
		
		int targetId = (allianceColor == MatchSettings.AllianceColor.BLUE ? 20 : 24);
		
		for (LLResultTypes.FiducialResult fiducial : limelightResult.getFiducialResults()) {
			if (fiducial.getFiducialId() == targetId) {
				// Get the 3D pose of the tag in camera space (meters)
				Pose3D cameraToTarget = fiducial.getTargetPoseCameraSpace();
				
				// Horizontal: yaw offset from camera center (tx)
				// This is the error that needs to be corrected
				double yawOffset = fiducial.getTargetXDegrees();
				
				// Convert to inches for our calculations
				double horizontalDistanceInches = cameraToTarget.getPosition().z * 39.3701;
				
				// Calculate height difference (launcher to goal)
				double heightDifferenceInches = Settings.Aiming.GOAL_HEIGHT_INCHES -
						Settings.Aiming.LAUNCHER_HEIGHT_INCHES;
				
				// Find optimal trajectory using projectile motion
				TrajectoryResult trajectory = calculateOptimalTrajectory(
						horizontalDistanceInches,
						heightDifferenceInches);
				
				if (trajectory == null) {
					return AimingSolution.invalid();
				}
				
				return new AimingSolution(
						yawOffset,
						trajectory.launchAngleDegrees,
						trajectory.launchVelocity,
						true);
			}
		}
		
		return AimingSolution.invalid();
	}
	
	/**
	 * Calculates the optimal launch trajectory using projectile motion physics.
	 * Tries to minimize impact velocity by preferring higher arc trajectories.
	 *
	 * @param horizontalDistance Distance to target in inches
	 * @param heightDifference   Height difference (target - launcher) in inches
	 * @return TrajectoryResult containing optimal angle and velocity, or null if no
	 * solution
	 */
	private TrajectoryResult calculateOptimalTrajectory(double horizontalDistance, double heightDifference) {
		double g = Settings.Aiming.GRAVITY_INCHES_PER_SEC_SQ;
		
		// Try different launch velocities from minimum to maximum
		TrajectoryResult bestTrajectory = null;
		double bestImpactScore = Double.POSITIVE_INFINITY;
		
		double minVelocity = Settings.Aiming.wheelRpmToVelocity(Settings.Aiming.MIN_WHEEL_SPEED_RPM);
		double maxVelocity = Settings.Aiming.wheelRpmToVelocity(Settings.Aiming.MAX_WHEEL_SPEED_RPM);
		
		// Sample velocities in steps
		int velocitySteps = 20;
		double velocityStep = (maxVelocity - minVelocity) / velocitySteps;
		
		for (int i = 0; i <= velocitySteps; i++) {
			double v0 = minVelocity + i * velocityStep;
			
			// Solve for launch angle using projectile motion equations
			// y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ))
			// Rearranging: tan²(θ) + tan(θ)*(-2*v₀²*x/(g*x²)) + (1 + 2*v₀²*y/(g*x²)) = 0
			
			double x = horizontalDistance;
			double y = heightDifference;
			double v0Sq = v0 * v0;
			
			// Quadratic coefficients for tan(θ)
			double a = g * x * x / (2 * v0Sq);
			double b = -x;
			double c = y + a;
			
			// Solve quadratic: a*tan²(θ) + b*tan(θ) + c = 0
			// Which becomes: tan²(θ) + (b/a)*tan(θ) + (c/a) = 0
			double A = 1.0;
			double B = b / a;
			double C = c / a;
			
			double discriminant = B * B - 4 * A * C;
			
			if (discriminant < 0) {
				continue; // No real solution for this velocity
			}
			
			// Two solutions: low angle and high angle
			double tanTheta1 = (-B + Math.sqrt(discriminant)) / (2 * A);
			double tanTheta2 = (-B - Math.sqrt(discriminant)) / (2 * A);
			
			// Convert to angles
			double angle1 = Math.toDegrees(Math.atan(tanTheta1));
			double angle2 = Math.toDegrees(Math.atan(tanTheta2));
			
			// Evaluate both solutions (prefer high angle for softer landing)
			TrajectoryResult traj1 = evaluateTrajectory(angle1, v0, x);
			TrajectoryResult traj2 = evaluateTrajectory(angle2, v0, x);
			
			if (traj1 != null && traj1.impactScore < bestImpactScore) {
				bestImpactScore = traj1.impactScore;
				bestTrajectory = traj1;
			}
			
			if (traj2 != null && traj2.impactScore < bestImpactScore) {
				bestImpactScore = traj2.impactScore;
				bestTrajectory = traj2;
			}
		}
		
		return bestTrajectory;
	}
	
	/**
	 * Evaluates a trajectory solution and calculates its impact characteristics.
	 *
	 * @param angleDegrees Launch angle in degrees
	 * @param velocity     Launch velocity in inches/second
	 * @param distance     Horizontal distance to target
	 * @return TrajectoryResult with impact metrics, or null if invalid
	 */
	private TrajectoryResult evaluateTrajectory(double angleDegrees, double velocity, double distance) {
		// Check if angle is within acceptable range
		if (angleDegrees < Settings.Aiming.MIN_LAUNCH_ANGLE_DEG ||
				angleDegrees > Settings.Aiming.MAX_LAUNCH_ANGLE_DEG) {
			return null;
		}
		
		double angleRad = Math.toRadians(angleDegrees);
		double g = Settings.Aiming.GRAVITY_INCHES_PER_SEC_SQ;
		
		// Calculate flight time
		double vx = velocity * Math.cos(angleRad);
		double vy = velocity * Math.sin(angleRad);
		double flightTime = distance / vx;
		
		// Check if flight time is reasonable
		if (flightTime > Settings.Aiming.MAX_FLIGHT_TIME_SEC || flightTime < 0) {
			return null;
		}
		
		// Calculate impact velocity components
		double impactVx = vx; // Horizontal velocity remains constant (ignoring air resistance)
		double impactVy = vy - g * flightTime; // Vertical velocity changes due to gravity
		
		// Apply simplified air resistance
		double velocityReduction = 1.0 - Settings.Aiming.AIR_RESISTANCE_FACTOR * flightTime;
		impactVx *= velocityReduction;
		impactVy *= velocityReduction;
		
		double impactSpeed = Math.sqrt(impactVx * impactVx + impactVy * impactVy);
		double impactAngleDeg = Math.abs(Math.toDegrees(Math.atan2(impactVy, impactVx)));
		
		// Calculate impact score (lower is better)
		// Prefer: high impact angle (steep entry), low impact speed, reasonable flight
		// time
		double angleScore = Math.abs(impactAngleDeg - Settings.Aiming.PREFERRED_IMPACT_ANGLE_DEG);
		double speedScore = impactSpeed / 100.0; // Normalize
		double timeScore = flightTime * 10.0; // Small penalty for long flight times
		
		double impactScore = angleScore + speedScore + timeScore;
		
		return new TrajectoryResult(angleDegrees, velocity, impactSpeed, impactAngleDeg,
				flightTime, impactScore);
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
	
	/**
	 * Internal class to hold trajectory calculation results.
	 */
	private static class TrajectoryResult {
		final double launchAngleDegrees;
		final double launchVelocity;
		final double impactSpeed;
		final double impactAngleDegrees;
		final double flightTime;
		final double impactScore; // Lower is better
		
		TrajectoryResult(double launchAngleDegrees, double launchVelocity,
		                 double impactSpeed, double impactAngleDegrees,
		                 double flightTime, double impactScore) {
			this.launchAngleDegrees = launchAngleDegrees;
			this.launchVelocity = launchVelocity;
			this.impactSpeed = impactSpeed;
			this.impactAngleDegrees = impactAngleDegrees;
			this.flightTime = flightTime;
			this.impactScore = impactScore;
		}
	}
}
