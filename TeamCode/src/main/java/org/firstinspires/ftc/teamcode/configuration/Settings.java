package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.software.Controller;

import java.util.EnumMap;

/**
 * The Settings class houses all constants and configurations for the robot.
 * This centralized approach makes tuning and adjustments more efficient.
 * The class is organized into logical static inner classes for clarity.
 */
@Configurable
public class Settings {
	/**
	 * Maps controller inputs to robot actions for TeleOp.
	 */
	@Configurable
	public static class Controls {
		public static final EnumMap<Controller.Action, Controller.Control> actionControlMap = new EnumMap<>(
				Controller.Action.class);
		
		static {
			// Main Controller (Driver)
			actionControlMap.put(Controller.Action.MOVE_Y, Controller.Control.LEFT_STICK_Y);
			actionControlMap.put(Controller.Action.MOVE_X, Controller.Control.LEFT_STICK_X);
			actionControlMap.put(Controller.Action.ROTATE_AXIS, Controller.Control.RIGHT_STICK_X);
			actionControlMap.put(Controller.Action.ROTATE_LEFT, Controller.Control.LEFT_BUMPER);
			actionControlMap.put(Controller.Action.ROTATE_RIGHT, Controller.Control.RIGHT_BUMPER);
			actionControlMap.put(Controller.Action.ABS_NORTH, Controller.Control.DPAD_UP);
			actionControlMap.put(Controller.Action.ABS_EAST, Controller.Control.DPAD_RIGHT);
			actionControlMap.put(Controller.Action.ABS_WEST, Controller.Control.DPAD_LEFT);
			actionControlMap.put(Controller.Action.ABS_SOUTH, Controller.Control.DPAD_DOWN);
			actionControlMap.put(Controller.Action.GOTO_CLOSE_SHOOT, Controller.Control.CIRCLE);
			actionControlMap.put(Controller.Action.GOTO_FAR_SHOOT, Controller.Control.CROSS);
			actionControlMap.put(Controller.Action.GOTO_HUMAN_PLAYER, Controller.Control.SQUARE);
			actionControlMap.put(Controller.Action.GOTO_GATE, Controller.Control.TRIANGLE);
			actionControlMap.put(Controller.Action.CANCEL_ASSISTED_DRIVING, Controller.Control.RIGHT_STICK_BUTTON);
			actionControlMap.put(Controller.Action.RESET_FOLLOWER, Controller.Control.BACK);
			actionControlMap.put(Controller.Action.TOGGLE_CENTRICITY, Controller.Control.LEFT_STICK_BUTTON);
			
			// Secondary Controller (Operator)
			actionControlMap.put(Controller.Action.AIM, Controller.Control.LEFT_TRIGGER);
			actionControlMap.put(Controller.Action.LAUNCH, Controller.Control.RIGHT_TRIGGER);
			actionControlMap.put(Controller.Action.LAUNCHER_STEEPNESS_AXIS, Controller.Control.RIGHT_STICK_Y);
			actionControlMap.put(Controller.Action.LAUNCHER_ROTATION_AXIS, Controller.Control.RIGHT_STICK_X);
			actionControlMap.put(Controller.Action.INTAKE, Controller.Control.SQUARE);
			actionControlMap.put(Controller.Action.RELEASE_EXTRAS, Controller.Control.CIRCLE);
			actionControlMap.put(Controller.Action.RELEASE_PURPLE, Controller.Control.TRIANGLE);
			actionControlMap.put(Controller.Action.RELEASE_GREEN, Controller.Control.CROSS);
			actionControlMap.put(Controller.Action.INCREMENT_CLASSIFIER_STATE, Controller.Control.DPAD_UP);
			actionControlMap.put(Controller.Action.EMPTY_CLASSIFIER_STATE, Controller.Control.DPAD_DOWN);
			
			for (Controller.Action action : Controller.Action.values()) {
				actionControlMap.putIfAbsent(action, Controller.Control.UNKNOWN);
			}
		}
	}
	
	/**
	 * Settings related to the robot's drivetrain and movement.
	 */
	@Configurable
	public static class Drive {
		// Multiplier applied to strafe movements to compensate for mechanical
		// differences
		public static final double STRAFE_POWER_COEFFICIENT = 1.2;
	}
	
	/**
	 * Parameters for assisted driving and alignment behaviors.
	 */
	@Configurable
	public static class Alignment {
		// Translational control
		public static double MAX_TRANSLATIONAL_SPEED = 0.5; // Max drive/strafe speed when far from target (0..1)
		public static double FULL_SPEED_DISTANCE = 30.0; // Distance (inches) outside of which translational speed hits
		// max
		public static double STOP_DISTANCE = 1.0; // Distance (inches) inside which translational speed tapers to near
		// zero
		
		// Rotational control
		public static double MAX_ROTATION_SPEED = 0.5; // Max rotation speed (0..1)
		public static double FULL_SPEED_HEADING_ERROR = Math.toRadians(90); // Heading error (radians) at which rotation
		// is full speed
		public static double HEADING_DEADBAND = Math.toRadians(2.5); // Deadband: don't rotate if error below this
		// (radians)
	}
	
	/**
	 * Hardware device name mapping.
	 */
	@Configurable
	public static class HardwareIDs {
		// Drive motors
		public static final String FRONT_LEFT_MOTOR = "frontLeft";
		public static final String FRONT_RIGHT_MOTOR = "frontRight";
		public static final String REAR_LEFT_MOTOR = "rearLeft";
		public static final String REAR_RIGHT_MOTOR = "rearRight";
		public static final String PINPOINT = "pinpoint"; // Assuming this is a drive motor or odometry pod
		
		// Subsystem motors and servos
		public static final String INTAKE_SERVO = "intakeServo";
		public static final String LAUNCHER_RIGHT = "launcherRight";
		public static final String LAUNCHER_LEFT = "launcherLeft";
		public static final String LAUNCHER_YAW_SERVO = "launcherYawServo";
		public static final String LAUNCHER_PITCH_SERVO = "launcherPitchServo";
		
		// Transfer mechanism
		public static final String TRANSFER_WHEEL_SERVO = "transferMainServo";
		public static final String TRANSFER_ENTRANCE_WHEEL = "transferEntranceServo"; // CR wheel at color sensor
		public static final String TRANSFER_EXIT_WHEEL = "transferExitServo"; // CR wheel at kicker position
		
		// Sensors
		public static final String TRANSFER_COLOR_SENSOR = "transferColorSensor";
		public static final String LIMELIGHT = "limelight";
	}
	
	/**
	 * Settings for the Intake mechanism.
	 */
	@Configurable
	public static class Intake {
		public static double SPEED = 0.5;
	}
	
	/**
	 * Settings for the Spindex (indexer/sorter) mechanism.
	 */
	@Configurable
	public static class Spindex {
		public static double[] SLOT_INTAKE_POSITIONS = {0.10, 0.43, 0.77}; // Calibrated servo positions for slots at
		// intake
		public static double EXIT_OFFSET = 0.25; // Offset from intake to exit alignment
		public static double RAPID_FIRE_COOLDOWN_MS = 200;
		public static long EJECT_EXIT_TIME_MS = 200; // ms for ball to fully leave the spindex after servo opens
		public static double EXIT_SERVO_CLOSED_POSITION = 1.0;
		public static double EXIT_SERVO_OPEN_POSITION = 0.0;
		public static double INTAKE_SERVO_CLOSED_POSITION = 1.0;
		public static double INTAKE_SERVO_OPEN_POSITION = 0.0;
		public static double TOLERANCE = 5.0 / 360.0; // how close a slot must be to the exit to launch
	}
	
	/**
	 * Settings for the transfer mechanism with CR management wheels.
	 * <p>
	 * The transfer has three wheels:
	 * - Main transfer wheel: moves balls through the transfer
	 * - Entrance wheel: CR wheel at color sensor that lets balls in
	 * - Exit wheel: CR wheel at kicker that fires balls out
	 */
	public static class Transfer {
		// Detection settings
		public static final double BLIND_WINDOW_MS = 200.0; // Time after detection to ignore new detections
		public static final int MAX_CAPACITY = 3; // Number of ball slots
		
		// Main transfer wheel settings
		public static final double TRANSFER_WHEEL_FORWARD_POWER = 0.5; // Power when advancing balls
		public static final double TRANSFER_WHEEL_REVERSE_POWER = -0.5; // Power when reversing
		public static final long TRANSFER_TIME_MS = 450; // Time to run wheel to move one ball slot
		
		// Entrance wheel settings (at color sensor position)
		public static final double ENTRANCE_WHEEL_INTAKE_POWER = -0.6; // Power when letting balls in
		public static final double ENTRANCE_WHEEL_HOLD_POWER = 0.15; // Small reverse to hold closed
		public static final long ENTRANCE_OPEN_DURATION_MS = 500; // How long to open entrance when intaking
		
		// Exit wheel settings (at kicker position)
		public static final double EXIT_WHEEL_FIRE_POWER = 1.0; // Full power when firing
		public static final double EXIT_WHEEL_HOLD_POWER = -0.15; // Small reverse to hold closed
		public static final long EXIT_FIRE_DURATION_MS = 300; // How long to spin exit wheel when firing
	}
	
	/**
	 * Settings for the Launcher mechanism.
	 * <p>
	 * Servo Angle System:
	 * - Yaw: 20° window (-10° to +10°), centered at 0° for camera offset
	 * corrections
	 * * 0° = center, -10° = left, +10° = right
	 * * Used for fine horizontal aiming adjustments
	 * <p>
	 * - Pitch: 90° window (0° to 90°), absolute angles from horizontal for launch
	 * physics
	 * * 0° = horizontal/parallel to ground
	 * * 45° = 45° launch angle
	 * * 90° = straight up
	 * * Matches projectile motion calculations in TrajectoryEngine
	 * <p>
	 * - Physical servo calibration points map these angle ranges to servo positions
	 *
	 * @noinspection PointlessBooleanExpression
	 */
	@Configurable
	public static class Launcher {
		public static long BELT_SPINUP_TIME_MS = 500;
		public static double BELT_SYNC_KP = 0.05; // Proportional gain for synchronizing belt speeds
		// Pitch servo calibration (physical limits)
		public static double PITCH_SERVO_AT_MIN = 0.692; // Servo position at minimum pitch angle
		public static double PITCH_SERVO_AT_MAX = 0.415; // Servo position at maximum pitch angle
		public static double DETECTION_PITCH = 25; // degrees from horizontal
		// Pitch angle window (absolute angles from horizontal, for launch physics)
		public static double PITCH_MIN_ANGLE = 0.0; // Minimum pitch angle in degrees (horizontal)
		public static double PITCH_MAX_ANGLE = 90.0; // Maximum pitch angle in degrees (straight up, 90° total window)
		// Yaw servo calibration (physical limits)
		public static double YAW_SERVO_AT_MIN = 0.0; // Servo position at minimum yaw angle
		public static double YAW_SERVO_AT_MAX = 1.0; // Servo position at maximum yaw angle
		// Yaw angle window (centered around 0°)
		public static double YAW_MIN_ANGLE = -10.0; // Minimum yaw angle in degrees
		public static double YAW_MAX_ANGLE = 10.0; // Maximum yaw angle in degrees (20° total window)
		public static boolean CORRECT_YAW = false && Deploy.LIMELIGHT;
		public static boolean CORRECT_PITCH = true;
		public static double[] OUTTAKE_DATA_X = {0.0, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0};
		public static double[] OUTTAKE_DATA_Y = {0.0, 90, 180, 257, 340, 423, 660};
		
		/**
		 * Converts pitch angle (degrees) to servo position using calibration points.
		 * Maps angle range [PITCH_MIN_ANGLE, PITCH_MAX_ANGLE] to servo range
		 * [PITCH_SERVO_AT_MIN, PITCH_SERVO_AT_MAX].
		 */
		public static double pitchToServo(double pitchDegrees) {
			// Clamp to valid angle range
			pitchDegrees = Math.max(PITCH_MIN_ANGLE, Math.min(PITCH_MAX_ANGLE, pitchDegrees));
			
			// Linear mapping: angle range to servo range
			double angleRange = PITCH_MAX_ANGLE - PITCH_MIN_ANGLE;
			double servoRange = PITCH_SERVO_AT_MAX - PITCH_SERVO_AT_MIN;
			double normalizedAngle = (pitchDegrees - PITCH_MIN_ANGLE) / angleRange;
			
			return PITCH_SERVO_AT_MIN + normalizedAngle * servoRange;
		}
		
		/**
		 * Converts servo position to pitch angle (degrees) using calibration points.
		 */
		public static double servoToPitch(double servoPosition) {
			// Inverse mapping: servo range to angle range
			double angleRange = PITCH_MAX_ANGLE - PITCH_MIN_ANGLE;
			double servoRange = PITCH_SERVO_AT_MAX - PITCH_SERVO_AT_MIN;
			double normalizedServo = (servoPosition - PITCH_SERVO_AT_MIN) / servoRange;
			
			return PITCH_MIN_ANGLE + normalizedServo * angleRange;
		}
		
		/**
		 * Converts yaw angle (degrees) to servo position using calibration points.
		 * Maps angle range [YAW_MIN_ANGLE, YAW_MAX_ANGLE] to servo range
		 * [YAW_SERVO_AT_MIN, YAW_SERVO_AT_MAX].
		 */
		public static double yawToServo(double yawDegrees) {
			// Clamp to valid angle range
			yawDegrees = Math.max(YAW_MIN_ANGLE, Math.min(YAW_MAX_ANGLE, yawDegrees));
			
			// Linear mapping: angle range to servo range
			double angleRange = YAW_MAX_ANGLE - YAW_MIN_ANGLE;
			double servoRange = YAW_SERVO_AT_MAX - YAW_SERVO_AT_MIN;
			double normalizedAngle = (yawDegrees - YAW_MIN_ANGLE) / angleRange;
			
			return YAW_SERVO_AT_MIN + normalizedAngle * servoRange;
		}
		
		/**
		 * Converts servo position to yaw angle (degrees) using calibration points.
		 */
		public static double servoToYaw(double servoPosition) {
			// Inverse mapping: servo range to angle range
			double angleRange = YAW_MAX_ANGLE - YAW_MIN_ANGLE;
			double servoRange = YAW_SERVO_AT_MAX - YAW_SERVO_AT_MIN;
			double normalizedServo = (servoPosition - YAW_SERVO_AT_MIN) / servoRange;
			
			return YAW_MIN_ANGLE + normalizedServo * angleRange;
		}
	}
	
	@Configurable
	public static class Vision {
		public static double LL_WINDOW_SIZE_DEGREES = 40; // Horizontal window size
	}
	
	@Configurable
	public static class ColorSensor {
		public static double[] GREEN_TARGET = {70, 200, 150};
		public static double[] PURPLE_TARGET = {120, 150, 220};
		public static double CONFIDENCE_THRESHOLD = 60.0; // Acceptable distance threshold
	}
	
	@Configurable
	public static class Aiming {
		public static double WHEEL_SPEED_OPTIMIZATION_STEP_RPM = 100.0;
		public static double MIN_ENTRY_ANGLE_DEGREES = 20.0;
		public static double MAX_LAUNCHER_ANGLE_DEGREES_FROM_HORIZONTAL = 75.0;
		// ===== Physical Measurements =====
		public static double LIMELIGHT_HEIGHT_INCHES = 8.25; // Height of limelight camera above field (when horizontal)
		public static double LAUNCHER_HEIGHT_INCHES = 12.25; // Height of launcher outtake above field
		public static double APRILTAG_CENTER_HEIGHT_INCHES = 29.5; // Height of AprilTag center above field
		public static double LIMELIGHT_FORWARD_OFFSET_INCHES = 1.5; // Limelight is 1.5 inches in front of launcher when
		// at horizontal
		public static double LIMELIGHT_VERTICAL_OFFSET_INCHES = 4.0; // Limelight is 4.0 inches below launcher when at
		// horizontal
		
		// ===== Pitch Axis Geometry =====
		// The launcher rotates around a pitch axis. Positions are relative to this
		// axis.
		public static double PITCH_AXIS_HEIGHT_INCHES = 10.5; // Height of pitch axis above field
		public static double PITCH_AXIS_FORWARD_OFFSET_INCHES = 0; // Pitch axis is directly below of launcher
		// outtake
		
		// Limelight position relative to pitch axis (rotates with launcher)
		public static double LIMELIGHT_FROM_PITCH_AXIS_FORWARD_INCHES = 1.5; // Limelight is 1.5 inches forward of pitch
		// when at horizontal
		// axis
		public static double LIMELIGHT_FROM_PITCH_AXIS_DOWN_INCHES = 2.2; // Limelight is 2.2 inches below pitch axis
		// when at horizontal
		
		// Launcher position relative to pitch axis (the outtake point)
		public static double LAUNCHER_FROM_PITCH_AXIS_BACK_INCHES = 0; // Launcher is directly on top axis when at 0°
		// from horizontal
		public static double LAUNCHER_FROM_PITCH_AXIS_UP_INCHES = 2.25; // Launcher is 2.25 inches above pitch axis
		// (radius from pitch axis)
		
		// ===== Simple Aiming Constants =====
		// Toggle between simple pose-based aiming and complex rotation-aware aiming
		public static boolean USE_COMPLEX_AIMING = true; // use conner's massive beautiful brain
		public static double TARGET_HEIGHT_OFFSET_INCHES = 50; // inches above apriltag to aim for
		
		/**
		 * Note that ROTATIONAL error refers to the chassis rotation relative to the
		 * goal.
		 * YAW refers to the launcher horizontal angle
		 * PITCH refers to the launcher vertical angle
		 */
		public static double MAX_ROTATIONAL_ERROR = Math.toRadians(20);
		public static double MAX_YAW_ERROR = 3.0; // degrees
		public static double MAX_PITCH_ERROR = 0.5; // degrees
		
		// ===== Complex Aiming Constants (Physics-Based) =====
		// Physical constants
		public static double GRAVITY_INCHES_PER_SEC_SQ = 386.4; // Standard gravity in inches/s²
		
		// Launcher specifications
		public static double WHEEL_DIAMETER_INCHES = 2.83; // Diameter of launcher wheels
		public static double DEFAULT_WHEEL_SPEED_RPM = 3000; // Default wheel speed in RPM
		public static double MIN_WHEEL_SPEED_RPM = 2500; // Minimum useful wheel speed
		public static double MAX_WHEEL_SPEED_RPM = 5000; // Maximum safe wheel speed
		public static double LAUNCH_EFFICIENCY = 0.85; // Energy transfer efficiency (0-1)
		
		// Launch geometry (legacy - kept for backwards compatibility)
		public static double GOAL_HEIGHT_INCHES = 37.5; // Height of goal center above field
	}
	
	/**
	 * Key locations on the game field.
	 */
	@Configurable
	public static class Field {
		public static double WIDTH = 144.0; // inches
		public static double BALL_MASS_KG = .076; // kg
		public static Pose RESET_POSE = new Pose(72, 72, Math.toRadians(270));
		public static Pose RED_GOAL_POSE = new Pose(130.0, 130.0, Math.toRadians(225));
		public static double[] RED_GOAL_AIM_3D = new double[]{RED_GOAL_POSE.getX(), RED_GOAL_POSE.getY(),
				7 + Aiming.GOAL_HEIGHT_INCHES};
		
		public static Pose BLUE_GOAL_POSE = new Pose(14.0, 130.0, Math.toRadians(315));
		public static double[] BLUE_GOAL_AIM_3D = new double[]{BLUE_GOAL_POSE.getX(), BLUE_GOAL_POSE.getY(),
				7 + Aiming.GOAL_HEIGHT_INCHES};
		public static Pose FAR_LAUNCH_ZONE_FRONT_CORNER = new Pose(72, 24);
		public static Pose FAR_LAUNCH_ZONE_LEFT_CORNER = new Pose(50, 0);
		public static Pose FAR_LAUNCH_ZONE_RIGHT_CORNER = new Pose(95, 0);
		
		public static Pose CLOSE_LAUNCH_ZONE_FRONT_CORNER = new Pose(72, 72);
		public static Pose CLOSE_LAUNCH_ZONE_LEFT_CORNER = new Pose(15, 128);
		public static Pose CLOSE_LAUNCH_ZONE_RIGHT_CORNER = new Pose(129, 128);
		
	}
	
	/**
	 * Flags to enable or disable major robot subsystems.
	 * Useful for testing and debugging.
	 *
	 * @noinspection ConstantValue, PointlessBooleanExpression
	 */
	@Configurable
	public static class Deploy {
		public static boolean INTAKE = true;
		public static boolean LIMELIGHT = true;
		public static boolean TRANSFER = true;
		public static boolean TRAJECTORY_ENGINE = true;
		public static boolean LAUNCHER = TRAJECTORY_ENGINE && true;
		
		public static boolean ALIGNMENT_ENGINE = true;
	}
	
	// A static class to hold all pose constants for organization.
	public static class Autonomous {
		// NOTE: BLUE alliance paths are the reference (we have a blue field for
		// tuning).
		// RED alliance paths are automatically mirrored by the PathRegistry.
		// Headings are in radians. 90 degrees = Math.toRadians(90)
		
		// Poses for the FAR side of the field, BLUE alliance TODO (REFERENCE - tune
		// these!)
		public static class BlueFar {
			public static Pose START = new Pose(65.533, 12.244, Math.toRadians(135));
			public static Pose PRESET_1_PREP = new Pose(35.526, 28.455, Math.toRadians(180));
			public static Pose PRESET_1_END = new Pose(18.453, 28.628, Math.toRadians(180));
			public static BezierCurve BEZIER_LAUNCH_1 = new BezierCurve(
					new Pose(18.453, 28.628),
					new Pose(64.671, 44.493),
					new Pose(63.808, 69.499));
			
			public static Pose ENDING_LAUNCH_1 = new Pose(63.808, 69.499, Math.toRadians(130));
			
			public static Pose PRESET_2_PREP = new Pose(38.802, 54.668, Math.toRadians(180));
			public static Pose PRESET_2_END = new Pose(18.970, 54.496, Math.toRadians(180));
			public static Pose LAUNCH_2 = new Pose(52, 80, Math.toRadians(135));
			public static Pose PRESET_3_END = new Pose(19.143, 80.019, Math.toRadians(180));
			public static Pose SCORE_3 = new Pose(40.354, 92.091, Math.toRadians(125));
			public static Pose PARK = new Pose(40.354, 92.091, Math.toRadians(125));
		}
		
		// Poses for the CLOSE side of the field, BLUE alliance TODO (REFERENCE - tune
		// these!)
		public static class BlueClose {
			// Start near the backdrop, facing forward.
			public static Pose START = new Pose(60, 85, Math.toRadians(135));
			// Position for the center spike mark.
			public static Pose PRESET_1_PREP = new Pose(37, 82.5, Math.toRadians(180));
			// Scoring position on the backdrop. Robot is flush, facing left.
			public static Pose PRESET_1_END = new Pose(15., 82.5, Math.toRadians(180));
			// A middle waypoint to help navigate under the stage truss.
			public static Pose LAUNCH_1 = new Pose(53, 85, Math.toRadians(120));
			// Position to pick up pixels from the stack across the field.
			public static Pose PRESET_2_PREP = new Pose(42, 57, Math.toRadians(180));
			// Scoring position on the backdrop. Robot is flush, facing left.
			public static Pose PRESET_2_END = new Pose(14, 57, Math.toRadians(180));
			// A middle waypoint to help navigate under the stage truss.
			public static Pose LAUNCH_2 = new Pose(66, 66, Math.toRadians(125));
			public static Pose PRESET_3_PREP = new Pose(41.5, 33, Math.toRadians(180));
			// Scoring position on the backdrop. Robot is flush, facing left.
			public static Pose PRESET_3_END = new Pose(13, 33, Math.toRadians(180));
			// A middle waypoint to help navigate under the stage truss.
			public static Pose LAUNCH_3 = new Pose(63, 20, Math.toRadians(120));
			public static Pose PARK = new Pose(63, 20, Math.toRadians(120));
		}
	}
}
