package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.software.Controller;

import java.util.EnumMap;

/**
 * The Settings class houses all constants and configurations for the robot.
 * This centralized approach makes tuning and adjustments more efficient.
 * The class is organized into logical static inner classes for clarity.
 */
@Configurable
public class Settings {
	public static class Color {
		public final static int RAGNAROK_RED = 0xFF0000;
		public final static int ELITE_GOLD = 0xFFEB29;
	}

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
			actionControlMap.put(Controller.Action.INTAKE_STOP, Controller.Control.CROSS);
			actionControlMap.put(Controller.Action.INTAKE_OUT, Controller.Control.CIRCLE);
			actionControlMap.put(Controller.Action.OVERRIDE_ADVANCE, Controller.Control.DPAD_UP);
			actionControlMap.put(Controller.Action.OVERRIDE_BALL_DETECTION, Controller.Control.DPAD_LEFT);
			actionControlMap.put(Controller.Action.OVERRIDE_CLEAR, Controller.Control.DPAD_RIGHT);
			actionControlMap.put(Controller.Action.OVERRIDE_REVERSE, Controller.Control.DPAD_DOWN);
			for (Controller.Action action : Controller.Action.values()) {
				actionControlMap.putIfAbsent(action, Controller.Control.UNKNOWN);
			}
		}
	}

	/**
	 * Hardware device name mapping.
	 * Each HardwareConfig stores both the device type and string name,
	 * allowing for type-safe hardware retrieval via the get() method.
	 */
	@Configurable
	public static class Hardware {
		// Drive motors
		public static final HardwareConfig FRONT_LEFT_MOTOR = new HardwareConfig(DcMotorEx.class, "frontLeft");
		public static final HardwareConfig FRONT_RIGHT_MOTOR = new HardwareConfig(DcMotorEx.class, "frontRight");
		public static final HardwareConfig REAR_LEFT_MOTOR = new HardwareConfig(DcMotorEx.class, "rearLeft");
		public static final HardwareConfig REAR_RIGHT_MOTOR = new HardwareConfig(DcMotorEx.class, "rearRight");
		public static final HardwareConfig PINPOINT = new HardwareConfig(GoBildaPinpointDriver.class, "pinpoint");

		// Subsystem motors and servos
		public static final HardwareConfig INTAKE_MOTOR = new HardwareConfig(DcMotorEx.class, "intakeMotor");
		public static final HardwareConfig LAUNCHER_RIGHT = new HardwareConfig(DcMotorEx.class, "launcherRight");
		public static final HardwareConfig LAUNCHER_LEFT = new HardwareConfig(DcMotorEx.class, "launcherLeft");
		public static final HardwareConfig LAUNCHER_YAW_SERVO = new HardwareConfig(ServoImplEx.class,
				"launcherYawServo");
		public static final HardwareConfig LAUNCHER_PITCH_SERVO = new HardwareConfig(ServoImplEx.class,
				"launcherPitchServo");

		// Transfer mechanism
		public static final HardwareConfig TRANSFER_WHEEL_SERVO = new HardwareConfig(CRServo.class,
				"transferMainServo");
		public static final HardwareConfig TRANSFER_EXIT_KICKER = new HardwareConfig(CRServo.class,
				"transferExitServo"); // CR servo at kicker position
		
		// Sensors (color sensor is now at intake location)
		public static final HardwareConfig TRANSFER_COLOR_SENSOR = new HardwareConfig(
				RevColorSensorV3.class, "transferColorSensor");
		public static final HardwareConfig LIMELIGHT = new HardwareConfig(Limelight3A.class, "limelight");

		/**
		 * Static method to retrieve hardware from a HardwareMap.
		 * Allows usage like: Settings.Hardware.get(FRONT_LEFT_MOTOR, hw)
		 *
		 * @param config      The hardware configuration
		 * @param hardwareMap The hardware map to retrieve from
		 * @param <T>         The type of hardware device to return
		 * @return The hardware device instance
		 */
		public static <T> T get(HardwareConfig config, HardwareMap hardwareMap) {
			return config.fromHardwareMap(hardwareMap);
		}
	}
	
	/**
	 * Settings for the Intake mechanism.
	 */
	@Configurable
	public static class Intake {
		// Color detection settings
		public static final long COLOR_DETECTION_DEBOUNCE_MS = 250; // Minimum time between detections
		public static final long BALL_TRAVEL_TIME_MS = 250; // Time for ball to travel from intake sensor to transfer
		public static double SPEED = 1.0;
		// entrance
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
	 * The transfer has two wheels:
	 * - Main transfer wheel: moves balls through the transfer
	 * - Exit wheel: CR wheel at kicker that fires balls out
	 * <p>
	 * Color detection and entrance control have been moved to the intake mechanism.
	 */
	public static class Transfer {
		public static final int MAX_CAPACITY = 3; // Number of ball slots
		
		// Main transfer wheel settings
		public static final double TRANSFER_WHEEL_FORWARD_POWER = 1.0; // Power when advancing balls
		public static final double TRANSFER_WHEEL_REVERSE_POWER = -1.0; // Power when reversing
		public static final long TRANSFER_TIME_MS = 900; // Time to run wheel to move one ball slot
		
		// Exit wheel settings (CR servo at kicker position)
		public static final double EXIT_FIRE_POWER = 0.0; // Power when firing ball out
		public static final double EXIT_HOLD_POWER = 1.0; // Power to hold closed (0 = stopped)
		public static final long EXIT_FIRE_DURATION_MS = 250; // How long to spin wheel to fire one ball
		
		// Automatic advance settings
		public static final boolean AUTO_ADVANCE_ENABLED = false; // Enable automatic ball advancement
		public static final long AUTO_ADVANCE_GRACE_PERIOD_MS = 500; // Wait time after detection before advancing
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
	
	/**
	 * Settings for the Brushland Labs Color Rangefinder sensor.
	 * Uses HSV-based hardware detection for lighting-independent color
	 * classification.
	 * <p>
	 * HSV hue values are scaled from 0-360° to 0-255 for sensor configuration.
	 * Purple: ~160-190° on color wheel
	 * Green: ~110-140° on color wheel
	 * <p>
	 * Documentation: https://docs.brushlandlabs.com/sensors/color-rangefinder/
	 */
	@Configurable
	public static class ColorSensor {
		// Purple HSV hue range (160-190 degrees scaled to 0-255)
		public static double PURPLE_HUE_LOW = 160.0 / 360.0 * 255.0; // ~113
		public static double PURPLE_HUE_HIGH = 190.0 / 360.0 * 255.0; // ~134
		
		// Green HSV hue range (110-140 degrees scaled to 0-255)
		public static double GREEN_HUE_LOW = 110.0 / 360.0 * 255.0; // ~78
		public static double GREEN_HUE_HIGH = 140.0 / 360.0 * 255.0; // ~99
		
		// Maximum detection distance (objects must be within this range)
		public static double MAX_DETECTION_DISTANCE_MM = 20.0; // millimeters
		
		// LEGACY
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
		public static double CLOSE_SHOOT_PITCH_DEGREES = 55.0; // Launch angle from horizontal for close position
		public static double CLOSE_SHOOT_RPM = 2500.0; // Wheel RPM for close position
		
		public static double FAR_SHOOT_PITCH_DEGREES = 53.0; // Launch angle from horizontal for far position
		public static double FAR_SHOOT_RPM = 3000.0; // Wheel RPM for far position
		
		/**
		 * Alignment tolerances.
		 * ROTATIONAL error refers to the chassis rotation relative to the goal.
		 * YAW refers to the launcher horizontal angle
		 * PITCH refers to the launcher vertical angle
		 */
		public static double MAX_ROTATIONAL_ERROR = Math.toRadians(2);
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
		
		/**
		 * Mirrors a pose across the field centerline for red alliance.
		 * Field width is 144 inches (standard FTC field).
		 * Takes a BLUE pose and returns the mirrored RED pose.
		 *
		 * @param bluePose The pose in BLUE alliance coordinates
		 * @return The mirrored pose in RED alliance coordinates
		 */
		public static Pose mirrorPose(Pose bluePose) {
			return new Pose(
					WIDTH - bluePose.getX(), // Mirror X coordinate
					bluePose.getY(), // Y stays the same
					Math.PI - bluePose.getHeading() // Mirror heading across x axis
			);
		}
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
			public static final Pose RESET = new Pose(134, 7, Math.toRadians(90)); // TODO set this to corner instead
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
			public static final Pose FAR_SHOOT = new Pose(60, 15, Math.toRadians(112));
			public static final Pose HUMAN_PLAYER = new Pose(30, 30, Math.toRadians(225));
			public static final Pose GATE = new Pose(25, 68, Math.toRadians(0));
			public static final Pose PARK = new Pose(105, 32, Math.toRadians(0));
		}
		
		/**
		 * Autonomous starting positions.
		 */
		public static class AutoStart {
			public static final Pose FAR = new Pose(56.25, 7.0, Math.toRadians(90));
			public static final Pose CLOSE = new Pose(21.84, 124.58, Math.toRadians(145));
		}
		
		/**
		 * Sample pickup locations organized by preset groups.
		 */
		public static class Samples {
			
			/**
			 * First preset group (closest to wall).
			 */
			public static class Preset1 {
				public static final Pose PREP = new Pose(40, 35.5, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(34.0, 35.5, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(28.0, 35.5, Math.toRadians(180));
				public static final Pose END = new Pose(18, 35.5, Math.toRadians(180));
			}
			
			/**
			 * Second preset group (middle).
			 */
			public static class Preset2 {
				public static final Pose PREP = new Pose(42, 58.5, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(34.0, 58.5, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(28.0, 58.5, Math.toRadians(180));
				public static final Pose END = new Pose(18, 58.5, Math.toRadians(180));
			}
			
			/**
			 * Third preset group (farthest from wall).
			 */
			public static class Preset3 {
				public static final Pose PREP = new Pose(40, 83, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(34.0, 83, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(28.0, 83, Math.toRadians(180));
				public static final Pose END = new Pose(18, 83, Math.toRadians(180));
			}
		}
		
		/**
		 * Control points for curved autonomous paths.
		 */
		public static class ControlPoints {
			// From sample areas to shooting positions
			public static final Pose CLOSE_LAUNCH_APPROACH_FAR = new Pose(67, 45);
			public static final Pose PRESET_1_APPROACH = new Pose(63, 35);
			public static final Pose CLOSE_TO_PRESET_2 = new Pose(64, 56);
			
			public static final Pose PRESET_2_APPROACH = new Pose(62, 59);
			
			public static final Pose FROM_PRESET3_TO_FAR = new Pose(52, 37);
		}
		
		/**
		 * Parking positions for end of autonomous.
		 */
		public static class Park {
			public static final Pose FAR = Samples.Preset1.GRAB_1; // Reuse a safe position
			public static final Pose CLOSE = Samples.Preset3.GRAB_1;
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
		public static boolean LIMELIGHT = false;
		public static boolean TRANSFER = true;
		public static boolean TRAJECTORY_ENGINE = true;
		public static boolean LAUNCHER = TRAJECTORY_ENGINE && true;
		
		public static boolean ALIGNMENT_ENGINE = true;
	}
	
	public static class Autonomous {
		public static double BALL_INTAKE_WAIT_S = 0.1;
		public static double SLOW_SPEED = 0.4;
		public static double LAUNCH_STABILITY_WAIT_S = 0.3;
		public static double MAX_ACTION_TIME_S = 4.5;
		
	}
}
