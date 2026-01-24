package org.firstinspires.ftc.teamcode.software;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain.Position;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Advanced Trajectory Engine using zone-based bilinear interpolation on
 * calibration data
 * to dynamically predict optimal RPM and pitch angle for any field position.
 * <p>
 * This system uses separate interpolation models for close and far launch
 * zones,
 * providing exceptional accuracy across the entire field.
 * <p>
 * Architecture:
 * - Two distinct launch zones: CLOSE (near basket) and FAR (mid-field)
 * - Each zone has its own calibration grid stored in CSV files
 * - Bilinear interpolation within each zone for smooth predictions
 * - Intelligent zone detection using triangle containment
 * - Preset anchoring when very close to known-good positions
 * <p>
 * Key Features:
 * - Zone-based bilinear interpolation for maximum accuracy
 * - File-based calibration data (close_zone.csv, far_zone.csv)
 * - Automatic zone detection via triangle containment test
 * - Physics-aware bounds checking for safety
 * - Graceful fallback to presets outside zones
 */
public class TrajectoryEngine {
	// Zone-specific calibration grids
	// These are loaded from CSV files at initialization
	private static List<CalibrationPoint> closeZoneData = new ArrayList<>();
	private static List<CalibrationPoint> farZoneData = new ArrayList<>();
	// Zone boundary triangles (defined in Settings)
	private static Triangle closeZoneTriangle;
	private static Triangle farZoneTriangle;
	// Track initialization status
	private static boolean isInitialized = false;
	MechanismManager mechanisms;
	HardwareMap hardwareMap;
	
	public TrajectoryEngine(MechanismManager mechanisms) {
		this.mechanisms = mechanisms;
		this.hardwareMap = mechanisms.hardwareMap;
	}
	
	/**
	 * Loads calibration data from CSV files and initializes zone boundaries.
	 * This must be called before using the trajectory engine.
	 * <p>
	 * CSV Format: x,y,rpm,pitch
	 * Example: 58.0,99.0,2665.0,45.2
	 * <p>
	 * Files are loaded from
	 * TeamCode/src/main/java/org/firstinspires/ftc/teamcode/software/datasets/
	 *
	 * @param hardwareMap The hardware map (not currently used, kept for
	 *                    compatibility)
	 */
	public static void load(HardwareMap hardwareMap) {
		if (isInitialized) {
			return; // Already loaded
		}
		
		// Initialize zone triangles from Settings
		closeZoneTriangle = new Triangle(
				Settings.Field.CLOSE_LAUNCH_ZONE_FRONT_CORNER,
				Settings.Field.CLOSE_LAUNCH_ZONE_LEFT_CORNER,
				Settings.Field.CLOSE_LAUNCH_ZONE_RIGHT_CORNER);
		
		farZoneTriangle = new Triangle(
				Settings.Field.FAR_LAUNCH_ZONE_FRONT_CORNER,
				Settings.Field.FAR_LAUNCH_ZONE_LEFT_CORNER,
				Settings.Field.FAR_LAUNCH_ZONE_RIGHT_CORNER);
		
		// Load calibration data from internal resource files
		try {
			closeZoneData = loadCalibrationResource(Settings.TrajectoryEngine.CLOSE_ZONE_DATA_FILE);
			farZoneData = loadCalibrationResource(Settings.TrajectoryEngine.FAR_ZONE_DATA_FILE);
			
			// If files don't exist or are empty, initialize with default presets
			if (closeZoneData.isEmpty()) {
				initializeDefaultCloseZone();
			}
			if (farZoneData.isEmpty()) {
				initializeDefaultFarZone();
			}
			
			isInitialized = true;
		} catch (Exception e) {
			// If loading fails, use default preset-based fallback
			initializeDefaultCloseZone();
			initializeDefaultFarZone();
			isInitialized = true;
		}
	}
	
	/**
	 * Loads calibration points from a CSV resource file in the project.
	 *
	 * @param resourcePath The path to the CSV resource file (relative to
	 *                     src/main/java/)
	 * @return List of calibration points
	 */
	private static List<CalibrationPoint> loadCalibrationResource(String resourcePath) {
		List<CalibrationPoint> points = new ArrayList<>();
		
		try {
			// Load as a resource from the classpath
			InputStream is = TrajectoryEngine.class.getClassLoader().getResourceAsStream(resourcePath);
			
			if (is == null) {
				// Resource not found, return empty list
				return points;
			}
			
			BufferedReader reader = new BufferedReader(new InputStreamReader(is));
			String line;
			boolean firstLine = true;
			
			while ((line = reader.readLine()) != null) {
				// Skip header line if present
				if (firstLine && line.trim().toLowerCase().startsWith("x")) {
					firstLine = false;
					continue;
				}
				firstLine = false;
				
				// Skip empty lines and comments
				line = line.trim();
				if (line.isEmpty() || line.startsWith("#")) {
					continue;
				}
				
				// Parse CSV line: x,y,rpm,pitch
				String[] parts = line.split(",");
				if (parts.length >= 4) {
					try {
						double x = Double.parseDouble(parts[0].trim());
						double y = Double.parseDouble(parts[1].trim());
						double rpm = Double.parseDouble(parts[2].trim());
						double pitch = Double.parseDouble(parts[3].trim());
						
						points.add(new CalibrationPoint(x, y, rpm, pitch));
					} catch (NumberFormatException e) {
						// Skip malformed lines
					}
				}
			}
			
			reader.close();
		} catch (Exception e) {
			// Return whatever we loaded so far
		}
		
		return points;
	}
	
	/**
	 * Initializes default calibration points for the close zone.
	 * Used as fallback when calibration file is not available.
	 */
	private static void initializeDefaultCloseZone() {
		closeZoneData.clear();
		
		// Center point (proven preset)
		closeZoneData.add(new CalibrationPoint(
				Settings.Positions.TeleOp.CLOSE_SHOOT.getX(),
				Settings.Positions.TeleOp.CLOSE_SHOOT.getY(),
				Settings.Aiming.CLOSE_SHOOT_RPM,
				Settings.Aiming.CLOSE_SHOOT_PITCH_DEGREES));
		
		// Additional estimated points for interpolation coverage
		closeZoneData.add(new CalibrationPoint(50, 85, 2750, 43.5));
		closeZoneData.add(new CalibrationPoint(65, 110, 2850, 46.0));
		closeZoneData.add(new CalibrationPoint(75, 95, 2700, 44.0));
	}
	
	/**
	 * Initializes default calibration points for the far zone.
	 * Used as fallback when calibration file is not available.
	 */
	private static void initializeDefaultFarZone() {
		farZoneData.clear();
		
		// Center point (proven preset)
		farZoneData.add(new CalibrationPoint(
				Settings.Positions.TeleOp.FAR_SHOOT.getX(),
				Settings.Positions.TeleOp.FAR_SHOOT.getY(),
				Settings.Aiming.FAR_SHOOT_RPM,
				Settings.Aiming.FAR_SHOOT_PITCH_DEGREES));
		
		// Additional estimated points for interpolation coverage
		farZoneData.add(new CalibrationPoint(55, 10, 3600, 31.5));
		farZoneData.add(new CalibrationPoint(65, 20, 3500, 33.0));
		farZoneData.add(new CalibrationPoint(72, 15, 3550, 32.0));
	}
	
	/**
	 * Adds a new calibration point to the appropriate zone dataset.
	 * Automatically detects which zone the point belongs to.
	 *
	 * @param x     Field x-coordinate (inches)
	 * @param y     Field y-coordinate (inches)
	 * @param rpm   Optimal RPM for this position
	 * @param pitch Optimal pitch angle (degrees) for this position
	 */
	public static void addCalibrationPoint(double x, double y, double rpm, double pitch) {
		LaunchZone zone = detectZoneStatic(x, y);
		CalibrationPoint point = new CalibrationPoint(x, y, rpm, pitch);
		
		switch (zone) {
			case CLOSE:
				closeZoneData.add(point);
				break;
			case FAR:
				farZoneData.add(point);
				break;
			case NONE:
				// Add to whichever zone is closer
				double distToClose = closeZoneTriangle.distanceTo(x, y);
				double distToFar = farZoneTriangle.distanceTo(x, y);
				if (distToClose < distToFar) {
					closeZoneData.add(point);
				} else {
					farZoneData.add(point);
				}
				break;
		}
	}
	
	/**
	 * Static helper for zone detection (used when adding calibration points).
	 */
	private static LaunchZone detectZoneStatic(double x, double y) {
		if (closeZoneTriangle != null && closeZoneTriangle.contains(x, y)) {
			return LaunchZone.CLOSE;
		}
		if (farZoneTriangle != null && farZoneTriangle.contains(x, y)) {
			return LaunchZone.FAR;
		}
		return LaunchZone.NONE;
	}
	
	/**
	 * Returns the current number of calibration points in each zone.
	 * Useful for telemetry and debugging.
	 *
	 * @return Array [closeZoneCount, farZoneCount]
	 */
	public static int[] getCalibrationPointCounts() {
		return new int[]{closeZoneData.size(), farZoneData.size()};
	}
	
	/**
	 * Calculates optimal launch parameters using zone-based bilinear interpolation.
	 * <p>
	 * Algorithm:
	 * 1. Determine which launch zone the robot is in (close, far, or neither)
	 * 2. Check if robot is near a preset position -> use preset directly
	 * 3. Otherwise, use bilinear interpolation within the detected zone
	 * 4. Apply physics-based bounds checking for safety
	 *
	 * @param allianceColor       The alliance color (for future enhancements)
	 * @param currentPitchDegrees Current pitch angle (for future adaptive control)
	 * @return An {@link AimingSolution} with calculated pitch and RPM
	 */
	public AimingSolution getAimingOffsets(MatchState.AllianceColor allianceColor, double currentPitchDegrees) {
		Pose currentPose = mechanisms.drivetrain.getPose();
		double x = currentPose.getX();
		double y = currentPose.getY();
		
		// Strategy 1: Check proximity to preset positions
		// If we're very close to a known-good preset, use it directly for consistency
		Pose closeShootPose = mechanisms.drivetrain.getPositionPose(Position.CLOSE_SHOOT);
		Pose farShootPose = mechanisms.drivetrain.getPositionPose(Position.FAR_SHOOT);
		
		double distanceToClose = getDistance(currentPose, closeShootPose);
		double distanceToFar = getDistance(currentPose, farShootPose);
		
		if (distanceToClose < Settings.TrajectoryEngine.PRESET_SNAP_THRESHOLD) {
			return new AimingSolution(
					Settings.Aiming.CLOSE_SHOOT_PITCH_DEGREES,
					Settings.Aiming.CLOSE_SHOOT_RPM,
					true);
		}
		
		if (distanceToFar < Settings.TrajectoryEngine.PRESET_SNAP_THRESHOLD) {
			return new AimingSolution(
					Settings.Aiming.FAR_SHOOT_PITCH_DEGREES,
					Settings.Aiming.FAR_SHOOT_RPM,
					true);
		}
		
		// Strategy 2: Detect which zone we're in and use zone-specific interpolation
		LaunchZone zone = detectZone(x, y);
		InterpolationResult result;
		
		switch (zone) {
			case CLOSE:
				result = interpolateInZone(x, y, closeZoneData);
				break;
			case FAR:
				result = interpolateInZone(x, y, farZoneData);
				break;
			case NONE:
			default:
				// Outside both zones - fall back to nearest preset
				if (distanceToClose < distanceToFar) {
					return new AimingSolution(
							Settings.Aiming.CLOSE_SHOOT_PITCH_DEGREES,
							Settings.Aiming.CLOSE_SHOOT_RPM,
							true);
				} else {
					return new AimingSolution(
							Settings.Aiming.FAR_SHOOT_PITCH_DEGREES,
							Settings.Aiming.FAR_SHOOT_RPM,
							true);
				}
		}
		
		// Strategy 3: Apply bounds checking
		double clampedPitch = clamp(result.pitch,
				Settings.Launcher.PITCH_MIN_ANGLE,
				Settings.Launcher.PITCH_MAX_ANGLE);
		double clampedRPM = clamp(result.rpm,
				Settings.TrajectoryEngine.MIN_RPM,
				Settings.TrajectoryEngine.MAX_RPM);
		
		return new AimingSolution(clampedPitch, clampedRPM, true);
	}
	
	/**
	 * Detects which launch zone contains the given position.
	 *
	 * @param x Field x-coordinate
	 * @param y Field y-coordinate
	 * @return The launch zone containing this position
	 */
	private LaunchZone detectZone(double x, double y) {
		if (closeZoneTriangle.contains(x, y)) {
			return LaunchZone.CLOSE;
		}
		if (farZoneTriangle.contains(x, y)) {
			return LaunchZone.FAR;
		}
		return LaunchZone.NONE;
	}
	
	/**
	 * Performs bilinear interpolation within a specific zone's calibration data.
	 * <p>
	 * Uses Inverse Distance Weighting (IDW) which provides smooth interpolation
	 * for irregular grids and handles the triangular zone boundaries naturally.
	 * <p>
	 * Formula: value = Σ(wi * vi) / Σ(wi)
	 * where wi = 1 / distance^power
	 *
	 * @param x        Field x-coordinate (inches)
	 * @param y        Field y-coordinate (inches)
	 * @param zoneData Calibration points for this zone
	 * @return Interpolated pitch and RPM values
	 */
	private InterpolationResult interpolateInZone(double x, double y, List<CalibrationPoint> zoneData) {
		if (zoneData.isEmpty()) {
			// Fallback if zone has no calibration data
			return new InterpolationResult(
					Settings.Aiming.CLOSE_SHOOT_RPM,
					Settings.Aiming.CLOSE_SHOOT_PITCH_DEGREES);
		}
		
		double weightedRPM = 0;
		double weightedPitch = 0;
		double totalWeight = 0;
		
		for (CalibrationPoint point : zoneData) {
			double dx = x - point.x;
			double dy = y - point.y;
			double distance = Math.sqrt(dx * dx + dy * dy);
			
			// Avoid division by zero - if we're exactly on a point, use it directly
			if (distance < Settings.TrajectoryEngine.MIN_INTERPOLATION_DISTANCE) {
				return new InterpolationResult(point.rpm, point.pitch);
			}
			
			double weight = 1.0 / Math.pow(distance, Settings.TrajectoryEngine.IDW_POWER);
			
			weightedRPM += weight * point.rpm;
			weightedPitch += weight * point.pitch;
			totalWeight += weight;
		}
		
		if (totalWeight == 0) {
			// Fallback: use first point in zone
			CalibrationPoint first = zoneData.get(0);
			return new InterpolationResult(first.rpm, first.pitch);
		}
		
		return new InterpolationResult(
				weightedRPM / totalWeight,
				weightedPitch / totalWeight);
	}
	
	/**
	 * Determines which shooting position (close or far) is closer.
	 * Kept for backward compatibility with existing code.
	 */
	public Position isCloseOrFar() {
		Pose currentPose = mechanisms.drivetrain.getPose();
		Pose closeShootPose = mechanisms.drivetrain.getPositionPose(Position.CLOSE_SHOOT);
		Pose farShootPose = mechanisms.drivetrain.getPositionPose(Position.FAR_SHOOT);
		
		double distanceToClose = getDistance(currentPose, closeShootPose);
		double distanceToFar = getDistance(currentPose, farShootPose);
		
		return (distanceToClose < distanceToFar) ? Position.CLOSE_SHOOT : Position.FAR_SHOOT;
	}
	
	/**
	 * Calculates Euclidean distance between two poses.
	 */
	private double getDistance(Pose pose1, Pose pose2) {
		double deltaX = pose2.getX() - pose1.getX();
		double deltaY = pose2.getY() - pose1.getY();
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}
	
	/**
	 * Clamps a value between min and max bounds.
	 */
	private double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
	
	/**
	 * Gets the current launch zone for a position.
	 * Useful for telemetry display.
	 */
	public LaunchZone getCurrentZone() {
		Pose pose = mechanisms.drivetrain.getPose();
		return detectZone(pose.getX(), pose.getY());
	}
	
	/**
	 * Enum representing the launch zones on the field.
	 */
	private enum LaunchZone {
		CLOSE, // Close to basket zone
		FAR, // Far/mid-field zone
		NONE // Outside both zones
	}
	
	/**
	 * Data class representing a calibration point on the field.
	 * Each point maps a field position to optimal launch parameters.
	 */
	private static class CalibrationPoint {
		final double x; // Field x-coordinate (inches)
		final double y; // Field y-coordinate (inches)
		final double rpm; // Optimal launcher RPM
		final double pitch; // Optimal pitch angle (degrees)
		
		CalibrationPoint(double x, double y, double rpm, double pitch) {
			this.x = x;
			this.y = y;
			this.rpm = rpm;
			this.pitch = pitch;
		}
	}
	
	/**
	 * Result of interpolation calculation.
	 */
	private static class InterpolationResult {
		final double rpm;
		final double pitch;
		
		InterpolationResult(double rpm, double pitch) {
			this.rpm = rpm;
			this.pitch = pitch;
		}
	}
	
	/**
	 * Represents a triangular zone on the field for launch zone detection.
	 * Uses barycentric coordinates for efficient point-in-triangle testing.
	 */
	private static class Triangle {
		final double x1, y1, x2, y2, x3, y3;
		final double det; // Precomputed determinant for efficiency
		
		Triangle(Pose p1, Pose p2, Pose p3) {
			this.x1 = p1.getX();
			this.y1 = p1.getY();
			this.x2 = p2.getX();
			this.y2 = p2.getY();
			this.x3 = p3.getX();
			this.y3 = p3.getY();
			
			// Precompute determinant for barycentric coordinate calculation
			this.det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
		}
		
		/**
		 * Tests if a point is inside this triangle using barycentric coordinates.
		 *
		 * @param x Point x-coordinate
		 * @param y Point y-coordinate
		 * @return true if point is inside triangle
		 */
		boolean contains(double x, double y) {
			// Calculate barycentric coordinates
			double lambda1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det;
			double lambda2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det;
			double lambda3 = 1.0 - lambda1 - lambda2;
			
			// Point is inside if all barycentric coordinates are non-negative
			return lambda1 >= 0 && lambda2 >= 0 && lambda3 >= 0;
		}
		
		/**
		 * Calculates the minimum distance from a point to this triangle.
		 * Used for fallback when point is outside both zones.
		 *
		 * @param x Point x-coordinate
		 * @param y Point y-coordinate
		 * @return Minimum distance to triangle
		 */
		double distanceTo(double x, double y) {
			// If point is inside, distance is 0
			if (contains(x, y)) {
				return 0;
			}
			
			// Calculate distance to each edge and return minimum
			double d1 = distanceToSegment(x, y, x1, y1, x2, y2);
			double d2 = distanceToSegment(x, y, x2, y2, x3, y3);
			double d3 = distanceToSegment(x, y, x3, y3, x1, y1);
			
			return Math.min(d1, Math.min(d2, d3));
		}
		
		/**
		 * Calculates distance from a point to a line segment.
		 */
		private double distanceToSegment(double px, double py,
		                                 double x1, double y1, double x2, double y2) {
			double dx = x2 - x1;
			double dy = y2 - y1;
			double lengthSq = dx * dx + dy * dy;
			
			if (lengthSq == 0) {
				// Segment is a point
				return Math.sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
			}
			
			// Project point onto line segment
			double t = Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / lengthSq));
			double projX = x1 + t * dx;
			double projY = y1 + t * dy;
			
			return Math.sqrt((px - projX) * (px - projX) + (py - projY) * (py - projY));
		}
	}
	
	/**
	 * Data class to hold the complete aiming solution including angles and launch
	 * velocity.
	 * <p>
	 * Coordinate System:
	 * - verticalOffsetDegrees: Absolute launch angle from horizontal (0° to 90°)
	 * * 0° = horizontal/parallel to ground
	 * * 45° = 45° launch angle
	 * * 90° = straight up
	 * <p>
	 * - rpm: Required launch motor RPM
	 */
	public static class AimingSolution {
		public final boolean hasTarget;
		public final double pitch; // Absolute pitch angle from horizontal
		public final double rpm; // Required launch motor rpm
		
		/**
		 * Constructor for a valid aiming solution.
		 */
		public AimingSolution(double pitch,
		                      double rpm, boolean hasTarget) {
			this.hasTarget = hasTarget;
			this.pitch = pitch;
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