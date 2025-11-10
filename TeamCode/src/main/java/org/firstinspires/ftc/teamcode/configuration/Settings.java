package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.configurables.annotations.Configurable;
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
		public static final Controller.Action[] gotoActions = {
				Controller.Action.GOTO_CLOSE_SHOOT,
				Controller.Action.GOTO_FAR_SHOOT,
				Controller.Action.GOTO_PARK,
				Controller.Action.GOTO_GATE
		};

		static {
			// Main Controller (Driver)
			actionControlMap.put(Controller.Action.MOVE_Y, Controller.Control.LEFT_STICK_Y);
			actionControlMap.put(Controller.Action.MOVE_X, Controller.Control.LEFT_STICK_X);
			actionControlMap.put(Controller.Action.ROTATE_AXIS, Controller.Control.RIGHT_STICK_X);
			actionControlMap.put(Controller.Action.ROTATE_LEFT, Controller.Control.LEFT_BUMPER);
			actionControlMap.put(Controller.Action.ROTATE_RIGHT, Controller.Control.RIGHT_BUMPER);
			actionControlMap.put(Controller.Action.SLOW_FORWARD, Controller.Control.DPAD_UP);
			actionControlMap.put(Controller.Action.SLOW_LEFT, Controller.Control.DPAD_RIGHT);
			actionControlMap.put(Controller.Action.SLOW_RIGHT, Controller.Control.DPAD_LEFT);
			actionControlMap.put(Controller.Action.SLOW_BACKWARD, Controller.Control.DPAD_DOWN);
			actionControlMap.put(Controller.Action.GOTO_CLOSE_SHOOT, Controller.Control.CROSS);
			actionControlMap.put(Controller.Action.GOTO_FAR_SHOOT, Controller.Control.TRIANGLE);
			actionControlMap.put(Controller.Action.GOTO_PARK, Controller.Control.SQUARE);
			actionControlMap.put(Controller.Action.GOTO_GATE, Controller.Control.CIRCLE);
			actionControlMap.put(Controller.Action.CANCEL_ASSISTED_DRIVING, Controller.Control.RIGHT_STICK_BUTTON);
			actionControlMap.put(Controller.Action.RESET_FOLLOWER, Controller.Control.BACK);
			actionControlMap.put(Controller.Action.TOGGLE_CENTRICITY, Controller.Control.LEFT_STICK_BUTTON);

			// Secondary Controller (Operator)
			actionControlMap.put(Controller.Action.AIM, Controller.Control.LEFT_TRIGGER);
			actionControlMap.put(Controller.Action.LAUNCH, Controller.Control.RIGHT_TRIGGER);
			actionControlMap.put(Controller.Action.LAUNCHER_STEEPNESS_AXIS, Controller.Control.RIGHT_STICK_Y);
			actionControlMap.put(Controller.Action.LAUNCHER_ROTATION_AXIS, Controller.Control.RIGHT_STICK_X);
			actionControlMap.put(Controller.Action.INTAKE_IN, Controller.Control.SQUARE);
			actionControlMap.put(Controller.Action.INTAKE_STOP, Controller.Control.CIRCLE);
			actionControlMap.put(Controller.Action.INTAKE_OUT, Controller.Control.CROSS);
			actionControlMap.put(Controller.Action.OVERRIDE_ADVANCE, Controller.Control.DPAD_UP);
			actionControlMap.put(Controller.Action.OVERRIDE_BALL_DETECTION, Controller.Control.DPAD_DOWN);
			actionControlMap.put(Controller.Action.OVERRIDE_SPINUP, Controller.Control.LEFT_BUMPER);
			for (Controller.Action action : Controller.Action.values()) {
				actionControlMap.putIfAbsent(action, Controller.Control.UNKNOWN);
			}
		}
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
		public static final String PINPOINT = "pinpoint";

		// Subsystem motors and servos
		public static final String INTAKE_MOTOR = "intakeMotor";
		public static final String LAUNCHER_RIGHT = "launcherRight";
		public static final String LAUNCHER_LEFT = "launcherLeft";
		public static final String LAUNCHER_YAW_SERVO = "launcherYawServo";
		public static final String LAUNCHER_PITCH_SERVO = "launcherPitchServo";

		// Transfer mechanism
		public static final String TRANSFER_WHEEL_SERVO = "transferMainServo";
		public static final String TRANSFER_ENTRANCE_WHEEL = "transferEntranceServo"; // CR wheel at color sensor
		public static final String TRANSFER_EXIT_KICKER = "transferExitServo"; // CR wheel at kicker position

		// Sensors
		public static final String TRANSFER_COLOR_SENSOR = "transferColorSensor";
		public static final String LIMELIGHT = "limelight";
	}

	/**
	 * Settings for the Intake mechanism.
	 */
	@Configurable
	public static class Intake {
		public static double SPEED = -1.0;
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
		public static double EXIT_SERVO_CLOSED_POSITION = 0.5;
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
		public static final double BLIND_WINDOW_MS = 750; // Time after detection to ignore new detections
		public static final int MAX_CAPACITY = 3; // Number of ball slots
		
		// Main transfer wheel settings
		public static final double TRANSFER_WHEEL_FORWARD_POWER = 1.0; // Power when advancing balls
		public static final double TRANSFER_WHEEL_REVERSE_POWER = -1.0; // Power when reversing
		public static final long TRANSFER_TIME_MS = 550; // Time to run wheel to move one ball slot
		
		// Entrance wheel settings (at color sensor position)
		public static final double ENTRANCE_WHEEL_INTAKE_POWER = 1.0; // Power when letting balls in
		public static final double ENTRANCE_WHEEL_HOLD_POWER = 0.0; // No reverse to hold closed
		public static final double ENTRANCE_WHEEL_OUT_POWER = -1.0;
		public static final long ENTRANCE_OPEN_DURATION_MS = 750; // How long to open entrance when intaking
		
		// Exit wheel settings (at kicker position)
		public static final double EXIT_KICK_POSITION = 0.4; // Launch
		public static final double EXIT_LOCK_POSITION = 1.0; // Closed
		public static final long EXIT_FIRE_DURATION_MS = 300; // How long it needs to fire
		public static final long EXIT_FIRE_RESET_MS = 300; // How long it needs to reset kicker to back position
		
		// Automatic advance settings
		public static final boolean AUTO_ADVANCE_ENABLED = true; // Enable automatic ball advancement
		public static final long AUTO_ADVANCE_GRACE_PERIOD_MS = 100; // Wait time after detection before auto-advancing
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
		public static long BELT_SPINUP_TIME_MS = 650;
		public static double BELT_SYNC_KP = 0.1; // Proportional gain for synchronizing belt speeds
		// Pitch servo calibration (physical limits)
		public static double PITCH_SERVO_AT_MIN = 0.692; // Servo position at minimum pitch angle
		public static double PITCH_SERVO_AT_MAX = 0.415; // Servo position at maximum pitch angle
		public static double DEFAULT_PITCH_ANGLE = 46.0; // degrees from horizontal
		// Pitch angle window (absolute angles from horizontal, for launch physics)
		public static double PITCH_MIN_ANGLE = 30.0; // Minimum pitch angle in degrees (horizontal)
		public static double PITCH_MAX_ANGLE = 80.0; // Maximum pitch angle in degrees (straight up, 90° total window)
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
		/**
		 * Preset launch angles and RPM for each shooting position.
		 * These values are used when AIM is called based on which position is closer.
		 */
		public static double CLOSE_SHOOT_PITCH_DEGREES = 54.0; // Launch angle from horizontal for close position
		public static double CLOSE_SHOOT_RPM = 2600.0; // Wheel RPM for close position
		
		public static double FAR_SHOOT_PITCH_DEGREES = 46; // Launch angle from horizontal for far position
		public static double FAR_SHOOT_RPM = 3500.0; // Wheel RPM for far position
		
		/**
		 * Alignment tolerances.
		 * ROTATIONAL error refers to the chassis rotation relative to the goal.
		 * YAW refers to the launcher horizontal angle
		 * PITCH refers to the launcher vertical angle
		 */
		public static double MAX_ROTATIONAL_ERROR = Math.toRadians(20);
		public static double MAX_YAW_ERROR = 3.0; // degrees
		public static double MAX_PITCH_ERROR = 0.5; // degrees
		
		// Legacy constants kept for compatibility
		public static double DEFAULT_WHEEL_SPEED_RPM = 4000; // Default wheel speed in RPM
		public static double MIN_WHEEL_SPEED_RPM = 2500; // Minimum useful wheel speed
		public static double MAX_WHEEL_SPEED_RPM = 5000; // Maximum safe wheel speed
		
		// Legacy constant for field geometry (used by Field class)
		public static double GOAL_HEIGHT_INCHES = 37.5; // Height of goal center above field
	}
	
	/**
	 * Field dimensions and physical constants.
	 */
	@Configurable
	public static class Field {
		public static double WIDTH = 144.0; // inches
		public static double BALL_MASS_KG = .076; // kg
	}
	
	/**
	 * Centralized collection of all robot positions on the field.
	 * All poses are defined in BLUE alliance coordinates and will be automatically
	 * mirrored by various systems when needed for RED alliance.
	 */
	@Configurable
	public static class Positions {
		
		/**
		 * Default/reset positions.
		 */
		public static class Default {
			public static final Pose RESET = new Pose(72, 72, Math.toRadians(90));
		}
		
		/**
		 * Goal positions for scoring.
		 */
		public static class Goals {
			public static final Pose RED_GOAL = new Pose(130.0, 130.0, Math.toRadians(225));
			public static final Pose BLUE_GOAL = new Pose(14.0, 130.0, Math.toRadians(315));
			
			// 3D aiming coordinates (x, y, z) for trajectory calculations
			public static final double[] RED_GOAL_AIM_3D = new double[]{
					RED_GOAL.getX(), RED_GOAL.getY(), 7 + Aiming.GOAL_HEIGHT_INCHES};
			public static final double[] BLUE_GOAL_AIM_3D = new double[]{
					BLUE_GOAL.getX(), BLUE_GOAL.getY(), 7 + Aiming.GOAL_HEIGHT_INCHES};
		}
		
		/**
		 * Launch zone boundaries for field awareness.
		 */
		public static class LaunchZones {
			// Far launch zone (closer to wall)
			public static final Pose FAR_FRONT_CORNER = new Pose(72, 24);
			public static final Pose FAR_LEFT_CORNER = new Pose(50, 0);
			public static final Pose FAR_RIGHT_CORNER = new Pose(95, 0);
			
			// Close launch zone (closer to goals)
			public static final Pose CLOSE_FRONT_CORNER = new Pose(72, 72);
			public static final Pose CLOSE_LEFT_CORNER = new Pose(15, 128);
			public static final Pose CLOSE_RIGHT_CORNER = new Pose(129, 128);
		}
		
		/**
		 * TeleOp operational positions (BLUE alliance reference).
		 * These are used for goto commands during driver control.
		 */
		public static class TeleOp {
			public static final Pose CLOSE_SHOOT = new Pose(58, 86, Math.toRadians(135));
			public static final Pose FAR_SHOOT = new Pose(60, 15, Math.toRadians(115));
			public static final Pose HUMAN_PLAYER = new Pose(30, 30, Math.toRadians(225));
			public static final Pose GATE = new Pose(25, 68, Math.toRadians(0));
			public static final Pose PARK = new Pose(104, 32, Math.toRadians(0));
		}
		
		/**
		 * Autonomous starting positions.
		 */
		public static class AutoStart {
			public static final Pose FAR = new Pose(56.25, 7.0, Math.toRadians(90));
			public static final Pose CLOSE = new Pose(22, 126, Math.toRadians(145));
		}
		
		/**
		 * Sample pickup locations organized by preset groups.
		 */
		public static class Samples {
			
			/**
			 * First preset group (closest to wall).
			 */
			public static class Preset1 {
				public static final Pose PREP = new Pose(43, 34, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 34, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 34, Math.toRadians(180));
				public static final Pose END = new Pose(20, 34, Math.toRadians(180));
			}
			
			/**
			 * Second preset group (middle).
			 */
			public static class Preset2 {
				public static final Pose PREP = new Pose(43, 60, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 60, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 60, Math.toRadians(180));
				public static final Pose END = new Pose(20, 60, Math.toRadians(180));
			}
			
			/**
			 * Third preset group (farthest from wall).
			 */
			public static class Preset3 {
				public static final Pose PREP = new Pose(43, 86, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(35.0, 86, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 86, Math.toRadians(180));
				public static final Pose END = new Pose(20, 86, Math.toRadians(180));
			}
		}
		
		/**
		 * Control points for curved autonomous paths.
		 */
		public static class ControlPoints {
			// From sample areas to shooting positions
			public static final Pose FROM_PRESET1_TO_CLOSE = new Pose(67, 45);
			public static final Pose FROM_PRESET2_TO_CLOSE = new Pose(64, 56);
			public static final Pose FROM_PRESET3_TO_CLOSE = new Pose(41, 81);
			public static final Pose FROM_PRESET3_TO_FAR = new Pose(52, 37);
		}
		
		/**
		 * Parking positions for end of autonomous.
		 */
		public static class Park {
			public static final Pose DEFAULT = Samples.Preset1.GRAB_1; // Reuse a safe position
		}
		
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
	
	// Autonomous settings - pose constants moved to AutonomousPoses class
	public static class Autonomous {
		public static double BALL_INTAKE_WAIT_S = 0.4;
	}
}
