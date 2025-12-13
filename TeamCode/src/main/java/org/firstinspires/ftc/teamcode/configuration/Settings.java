package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
		public static final HardwareConfig INTAKE_MOTOR = new HardwareConfig(DcMotorEx.class, "intake");
		public static final HardwareConfig LAUNCHER_RIGHT = new HardwareConfig(DcMotorEx.class, "launcherRight");
		public static final HardwareConfig LAUNCHER_LEFT = new HardwareConfig(DcMotorEx.class, "launcherLeft");
		
		public static final HardwareConfig LAUNCHER_PITCH_SERVO = new HardwareConfig(ServoImplEx.class,
				"launcherPitch");
		public static final HardwareConfig LAUNCHER_WALL_LEFT = new HardwareConfig(ServoImplEx.class,
				"launcherWallRight");
		public static final HardwareConfig LAUNCHER_WALL_RIGHT = new HardwareConfig(ServoImplEx.class,
				"launcherWallLeft");

		// Transfer mechanism
		public static final HardwareConfig TRANSFER_WHEEL_MOTOR = new HardwareConfig(DcMotorEx.class,
				"transfer");
		
		
		public static final HardwareConfig COMPARTMENT_LEFT = new HardwareConfig(ServoImplEx.class,
				"compartmentLeft");
		
		public static final HardwareConfig COMPARTMENT_RIGHT = new HardwareConfig(ServoImplEx.class,
				"compartmentLeft");
		
		
		// Sensors
		public static final String[] COLOR_RANGEFINDER_1 = {"crf1_0", "crf1_1"};
		public static final String[] COLOR_RANGEFINDER_2 = {"crf2_0", "crf2_1"};
		public static final HardwareConfig CONFIGURE_COLOR_SENSOR = new HardwareConfig(
				RevColorSensorV3.class, "colorSensorConfigure");

		
		
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
		public static double IN_SPEED = 1.0;
		public static double OUT_SPEED = -1.0;
		public static double STOPPED_SPEED = 0.0;
		public static long COLOR_SENSOR_DEBOUNCE_TIME = 500;
	}
	
	/**
	 * Settings for the main transfer wheel.
	 */
	public static class Transfer {
		public static final double SPEED = 1.0;
		public static final int FIRING_POSITION_TICKS = 1000; // TODO
		public static final double INCREMENT_MS = 300;
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
		public static long BELT_SPINUP_TIME_MS = 400;
		public static double CARTRIDGE_OPEN_POS = 1.0;
		public static double CARTRIDGE_CLOSED_POS = 0.0;
		public static long TICKS_PER_REVOLUTION = 28;
		public static long MAX_SPEED_ERROR = 100;
		public static long EXIT_FIRE_DURATION_MS = 250;
		public static long EXIT_FIRE_RESET_MS = 250;

		
		// Pitch servo calibration (physical limits)
		public static double PITCH_SERVO_AT_MIN = 0.692; // Servo position at minimum pitch angle
		public static double PITCH_SERVO_AT_MAX = 0.415; // Servo position at maximum pitch angle
		public static double DEFAULT_PITCH_ANGLE = 46.0; // degrees from horizontal
		public static double PITCH_MIN_ANGLE = 30.0; // Minimum pitch angle in degrees (horizontal)
		public static double PITCH_MAX_ANGLE = 80.0; // Maximum pitch angle in degrees (straight up, 90° total window)
		
		
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
		public static double CLOSE_SHOOT_RPM = 2600.0; // Wheel RPM for close position
		
		public static double FAR_SHOOT_PITCH_DEGREES = 53.0; // Launch angle from horizontal for far position
		public static double FAR_SHOOT_RPM = 3150.0; // Wheel RPM for far position
		
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
				public static final Pose PREP = new Pose(45, 34, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 34, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 34, Math.toRadians(180));
				public static final Pose END = new Pose(20, 34, Math.toRadians(180));
			}
			
			/**
			 * Second preset group (middle).
			 */
			public static class Preset2 {
				public static final Pose PREP = new Pose(45, 58, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 58, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 58, Math.toRadians(180));
				public static final Pose END = new Pose(20, 58, Math.toRadians(180));
			}
			
			/**
			 * Third preset group (farthest from wall).
			 */
			public static class Preset3 {
				public static final Pose PREP = new Pose(45, 83, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(35.0, 83, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 83, Math.toRadians(180));
				public static final Pose END = new Pose(20, 83, Math.toRadians(180));
			}
		}
		
		/**
		 * Control points for curved autonomous paths.
		 */
		public static class ControlPoints {
			// From sample areas to shooting positions
			public static final Pose CLOSE_LAUNCH_APPROACH_FAR = new Pose(67, 45);
			public static final Pose PRESET_1_APPROACH = new Pose(63, 35);
			public static final Pose FROM_PRESET2_TO_CLOSE = new Pose(64, 56);
			public static final Pose PRESET_2_APPROACH = new Pose(62, 59);
			
			public static final Pose FROM_PRESET3_TO_CLOSE = new Pose(41, 81);
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
		
		public static boolean COMPARTMENT = true;
	}
	
	public static class Autonomous {
		public static double BALL_INTAKE_WAIT_S = 0.35;
		public static double SLOW_SPEED = 0.27;
		public static double LAUNCH_STABILITY_WAIT_S = 0.3;
		public static double MAX_ACTION_TIME_S = 4.5;
		
	}
}
