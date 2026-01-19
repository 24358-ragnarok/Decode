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
	public static class Robot {
		public static final double WIDTH = 16.25;
		public static final double LENGTH = 17.00;

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
			actionControlMap.put(Controller.Action.SET_FOLLOWER, Controller.Control.PS);
			actionControlMap.put(Controller.Action.TOGGLE_CENTRICITY, Controller.Control.LEFT_STICK_BUTTON);

			// Secondary Controller (Operator)
			actionControlMap.put(Controller.Action.AIM, Controller.Control.LEFT_TRIGGER);
			actionControlMap.put(Controller.Action.LAUNCH, Controller.Control.RIGHT_TRIGGER);
			actionControlMap.put(Controller.Action.LAUNCHER_STEEPNESS_AXIS, Controller.Control.RIGHT_STICK_Y);
			actionControlMap.put(Controller.Action.LAUNCHER_ROTATION_AXIS, Controller.Control.RIGHT_STICK_X);
			actionControlMap.put(Controller.Action.INTAKE_IN, Controller.Control.CROSS);
			actionControlMap.put(Controller.Action.INTAKE_OUT, Controller.Control.TRIANGLE);
			actionControlMap.put(Controller.Action.TRANSFER_ADVANCE, Controller.Control.DPAD_UP);
			actionControlMap.put(Controller.Action.OVERRIDE_BALL_DETECTION, Controller.Control.DPAD_LEFT);
			actionControlMap.put(Controller.Action.OVERRIDE_CLEAR, Controller.Control.DPAD_RIGHT);
			actionControlMap.put(Controller.Action.TRANSFER_REVERSE, Controller.Control.DPAD_DOWN);
			actionControlMap.put(Controller.Action.PARK_EXTEND_1, Controller.Control.OPTIONS);
			actionControlMap.put(Controller.Action.PARK_EXTEND_2, Controller.Control.BACK);
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
				"pitch");
		public static final HardwareConfig LAUNCHER_GATE = new HardwareConfig(ServoImplEx.class,
				"gate");

		// Transfer mechanism
		public static final HardwareConfig TRANSFER_WHEEL_MOTOR = new HardwareConfig(DcMotorEx.class,
				"transfer");

		public static final HardwareConfig SWAP = new HardwareConfig(ServoImplEx.class,
				"swap");

		public static final HardwareConfig EXTENDER_LEFT = new HardwareConfig(ServoImplEx.class,
				"extenderLeft");
		public static final HardwareConfig EXTENDER_RIGHT = new HardwareConfig(ServoImplEx.class,
				"extenderRight");

		// Sensors
		public static final String[] COLOR_RANGEFINDER_1 = {"crf1_0", "crf1_1"};
		public static final String[] COLOR_RANGEFINDER_2 = {"crf2_0", "crf2_1"};
		public static final HardwareConfig COLOR_SENSOR_LEFT = new HardwareConfig(RevColorSensorV3.class,
				"colorLeft");
		public static final HardwareConfig COLOR_SENSOR_RIGHT = new HardwareConfig(RevColorSensorV3.class,
				"colorRight");
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
		public static double CRAWL_SPEED = 0.8;
		public static double OUT_SPEED = -1.0;
		public static double STOPPED_SPEED = 0.0;
		public static long COLOR_SENSOR_DEBOUNCE_TIME_MS = 200;
	}
	
	/**
	 * Settings for the main transfer wheel.
	 * <p>
	 * Uses motion-profiled position control for smooth, oscillation-free movement.
	 * The controller uses a trapezoidal velocity profile to accelerate/decelerate
	 * smoothly, preventing jerky movements that could jam balls.
	 * <p>
	 * Tuning Guide (see TUNING.md for detailed instructions):
	 * - kP: Start low (0.01), increase until response is crisp but not oscillating
	 * - kD: Add damping if oscillation occurs (typically 2-10x kP)
	 * - maxVelocity: Maximum ticks/sec during movement
	 * - maxAcceleration: How fast to ramp velocity (lower = smoother)
	 * - tolerance: Position deadband to prevent hunting
	 */
	@Configurable
	public static class Transfer {
		public static final double FIRING_POSITION_TICKS = 900;
		public static final double INCREMENT_TICKS = FIRING_POSITION_TICKS / 1.5;
		public static final double DECREMENT_TICKS = -FIRING_POSITION_TICKS / 1.5;
		public static double SPEED = 1.0;
		public static double CRAWL_SPEED = 0.3;
		public static double CRAWL_TICKS = FIRING_POSITION_TICKS * 3;
		// Motion Profile Position Controller Gains
		public static double TRANSFER_MOTOR_SPINUP_MS = 100;
		public static int POSITION_TOLERANCE = 20; // Acceptable position error (ticks)
	}
	
	@Configurable
	public static class Lever {
		public static double RIGHT_SERVO_DEPLOY_POS = 0.279;
		public static double RIGHT_SERVO_RETRACTED_POS = 0.0;
		public static double LEFT_SERVO_DEPLOY_POS = 0.176;
		public static double LEFT_SERVO_RETRACTED_POS = 0.0;
	}
	
	/**
	 * Settings for the Launcher mechanism.
	 * <p>
	 * Uses PIDF velocity control for maximum spinup speed while maintaining
	 * accuracy.
	 * The feedforward (kF) term provides the base power needed for a given RPM,
	 * while P/I/D terms correct for load variations and friction.
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
	 * <p>
	 * PIDF Tuning Guide (see TUNING.md for detailed instructions):
	 * - kF: Feedforward - set first, should get you ~90% of target speed
	 * - kP: Proportional - increase for faster response, decrease if oscillating
	 * - kI: Integral - use sparingly, eliminates steady-state error
	 * - kD: Derivative - dampens oscillation, typically 10x kP
	 *
	 * @noinspection PointlessBooleanExpression
	 */
	@Configurable
	public static class Launcher {
		public static final double TICKS_PER_REVOLUTION = 28.0;
		public static final double VELOCITY_ALPHA = 0.2; // EMA smoothing factor (0-1), lower = more smoothing
		public static double GATE_FIRE_POSITION_CLOSE = 0.388;
		public static double GATE_FIRE_POSITION_FAR = 0.404;
		public static double GATE_CLOSED_POSITION = 0.444;
		public static long GATE_COOLDOWN_MS = 500;
		public static long MAX_SPEED_ERROR = 30;
		// Pitch servo calibration (physical limits)
		public static double PITCH_SERVO_AT_MIN = 0.400; // Servo position at minimum pitch angle
		public static double PITCH_SERVO_AT_MAX = 0.800; // Servo position at maximum pitch angle
		public static double DEFAULT_PITCH_ANGLE = 40.0; // degrees from horizontal;
		// fixes ts
		public static double PITCH_MIN_ANGLE = 24.0; // Minimum pitch angle in degrees (horizontal)
		public static double PITCH_MAX_ANGLE = 53.0; // Maximum pitch angle in degrees (straight up, 90° total window)

		/**
		 * Converts RPM to motor velocity in ticks per second.
		 * Use this when setting motor velocity from an RPM value.
		 *
		 * @param rpm revolutions per minute
		 * @return velocity in ticks per second
		 */
		public static double rpmToTicksPerSec(double rpm) {
			return rpm * TICKS_PER_REVOLUTION / 60.0;
		}
		
		/**
		 * Converts motor velocity in ticks per second to RPM.
		 * Use this when reading motor velocity and displaying as RPM.
		 *
		 * @param ticksPerSec velocity in ticks per second
		 * @return revolutions per minute
		 */
		public static double ticksPerSecToRPM(double ticksPerSec) {
			return ticksPerSec / TICKS_PER_REVOLUTION * 60.0;
		}
		
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
		
		// Normalized RGB thresholds (values centered around 1.0)
		public static double GREEN_THRESHOLD = 1.4; // Normalized green channel threshold for GREEN detection
		public static double BLUE_THRESHOLD = 1.2; // Normalized blue channel threshold for PURPLE detection
	}
	
	@Configurable
	public static class Aiming {
		/**
		 * Preset launch angles and RPM for each shooting position.
		 * These values are used when AIM is called based on which position is closer.
		 */
		public static double CLOSE_SHOOT_PITCH_DEGREES = 45.2; // Launch angle from horizontal for close position
		public static double CLOSE_SHOOT_RPM = 2665; // Wheel RPM for close position
		
		public static double FAR_SHOOT_PITCH_DEGREES = 32.5; // Launch angle from horizontal for far position
		public static double FAR_SHOOT_RPM = 3605; // Wheel RPM for far position

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
		 * Reset/reset positions.
		 */
		public static class Reset {
			public static final Pose HUMAN_PLAYER_ZONE = new Pose(144 - (Robot.WIDTH / 2), Robot.LENGTH / 2,
					Math.toRadians(90));
			public static final Pose FAR_ZONE = new Pose(59.00, Robot.LENGTH / 2, Math.toRadians(90));
			public static final Pose CLOSE_ZONE = new Pose(34, 144 - (Robot.LENGTH / 2), Math.toRadians(90));
		}
		
		/**
		 * Goal positions for scoring.
		 */
		public static class Towers {
			public static final Pose RED_GOAL = new Pose(130.0, 130.0, Math.toRadians(225));
			public static final Pose BLUE_GOAL = new Pose(14.0, 130.0, Math.toRadians(315));
			public static final Pose OBELISK = new Pose(72.0, 150.0, Math.toRadians(0));
			// Close scan matches the previous single SCAN pose for backward compatibility.
			public static final Pose CLOSE_SCAN = new Pose(60, 100.0, Math.toRadians(80));
			// Far scan can be tuned separately; initialized to the same pose for now.
			public static final Pose FAR_SCAN = new Pose(60, 12, Math.toRadians(80));
			// Legacy alias retained for any existing callers.
			public static final Pose SCAN = CLOSE_SCAN;
		}
		
		/**
		 * TeleOp operational positions (BLUE alliance reference).
		 * These are used for goto commands during driver control.
		 */
		public static class TeleOp {
			public static final Pose CLOSE_SHOOT = new Pose(58, 99, Math.toRadians(140.59));
			public static final Pose FAR_SHOOT = new Pose(60, 18, Math.toRadians(114));
			public static final Pose HUMAN_PLAYER = new Pose(30, 30, Math.toRadians(225));
			public static final Pose GATE = new Pose(25, 73, Math.toRadians(270));
			public static final Pose PARK = new Pose(106, 32, Math.toRadians(180));
		}
		
		/**
		 * Autonomous starting positions.
		 */
		public static class AutoStart {
			public static final Pose FAR = new Pose(59.00, Robot.LENGTH / 2, Math.toRadians(90));
			public static final Pose CLOSE = new Pose(26, 124, Math.toRadians(145));
		}
		
		/**
		 * Sample pickup locations organized by preset groups.
		 */
		public static class Samples {
			public static Pose EAT = new Pose(16, 30, Math.toRadians(115));
			public static Pose EAT_END = new Pose(16, 55, Math.toRadians(115));
			/**
			 * First preset group (closest to wall).
			 */
			public static class Preset1 {
				public static final Pose PREP = new Pose(44, 34, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 34, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 34, Math.toRadians(180));
				public static final Pose END = new Pose(20, 34, Math.toRadians(180));
			}
			
			/**
			 * Second preset group (middle).
			 */
			public static class Preset2 {
				public static final Pose PREP = new Pose(44, 58, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 58, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 58, Math.toRadians(180));
				public static final Pose END = new Pose(25, 58, Math.toRadians(180));
			}
			
			/**
			 * Third preset group (farthest from wall).
			 */
			public static class Preset3 {
				public static final Pose PREP = new Pose(44, 84, Math.toRadians(180));
				public static final Pose GRAB_1 = new Pose(36.0, 84, Math.toRadians(180));
				public static final Pose GRAB_2 = new Pose(30.0, 84, Math.toRadians(180));
				public static final Pose END = new Pose(20, 84, Math.toRadians(180));
			}
			
			public static class HumanPlayerPreset {
				public static final Pose PREP = new Pose(16.0, 35.16, Math.toRadians(-90));
				public static final Pose END = new Pose(13.0, 13.0, Math.toRadians(-90));
			}
		}
		
		/**
		 * Control points for curved autonomous paths.
		 */
		public static class ControlPoints {
			// From sample areas to shooting positions
			public static final Pose PRESET_1_APPROACH_FAR = new Pose(75, 38);
			public static final Pose FROM_PRESET2_TO_CLOSE = new Pose(64, 56);
			public static final Pose PRESET_2_APPROACH_FAR = new Pose(65, 59);
			
			public static final Pose FROM_PRESET3_TO_CLOSE = new Pose(41, 81);
			public static final Pose FROM_CLOSE_SHOOT_TO_PRESET2 = new Pose(41, 59);
			public static final Pose FROM_CLOSE_SHOOT_TO_PRESET3 = new Pose(96, 81);
			public static final Pose FROM_PRESET3_TO_FAR = new Pose(52, 37);
			public static Pose EAT = new Pose(17, 20);
		}
		
		/**
		 * Parking positions for end of autonomous.
		 */
		public static class Park {
			public static final Pose FAR = Samples.Preset1.GRAB_1; // Reuse a safe position
			public static final Pose CLOSE = new Pose(30, 73, Math.toRadians(270));
			public static final Pose FAR_SAFE_PARK_POSE = new Pose(35.86206896551724, 12.505747126436786,
					Math.toRadians(90));
			public static final Pose CLOSE_SAFE_PARK_POSE = new Pose(48, 130, Math.toRadians(90));
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
		
		public static boolean SWAP = true;
		public static boolean LEVER = false;
	}
	
	public static class Swap {
		public static double GRABBING_POS = 0.547;
		public static double HOLDING_POS = 0.834;
		public static long COOLDOWN_MS = 500;
		
	}
	
	public static class Autonomous {
		/**
		 * Total autonomous period duration in seconds (FTC standard is 30 seconds).
		 */
		public static double AUTO_PERIOD_SECONDS = 30.0;
		
		public static double SLOW_SPEED = 0.4;
		public static double LAUNCH_EXIT_TIME_MS = 200;
		public static double KRAKATOA_TIME_MS = 1500;
		public static double MAX_ACTION_TIME_S = 10.0;
		public static double SEARCH_TIMEOUT_MS = 500; // Time to search before assuming empty
		
		/**
		 * Per-action timeout configuration.
		 * Each action type can have its own timeout (in seconds).
		 * Set to 0 to disable timeout for that action type.
		 * <p>
		 * Timeouts are keyed by action class simple name (e.g., "LaunchAction",
		 * "LinearPathAction").
		 * <p>
		 * When an action's timeout is exceeded, it will be immediately ended with
		 * interrupted=true.
		 */
		@Configurable
		public static class ActionTimeouts {
			// Path actions
			public static double LinearPathAction = 0.0; // No timeout (default)
			public static double SlowLinearPathAction = 0.0;
			public static double SplinedPathAction = 0.0;
			public static double CurvePathAction = 0.0;
			
			// Launch actions
			public static double LaunchAction = 0.0;
			public static double SortedLaunchAction = 0.0;
			public static double PrepareLaunchAction = 0.0;
			
			// Pickup actions
			public static double PickupBallAction = 0.0;
			public static double EndPickupAction = 0.0;
			
			// Utility actions
			public static double WaitAction = 0.0;
			public static double ScanAction = 0.0;
			public static double StartAtAction = 0.0;
			public static double EndAtAction = 0.0;
			public static double ParallelAction = 0.0;
			
			/**
			 * Gets the timeout for an action by its class name.
			 * Returns 0 if the action type is not configured (no timeout).
			 *
			 * @param actionClassName The simple class name of the action (e.g.,
			 *                        "LaunchAction")
			 * @return Timeout in seconds (0 = no timeout)
			 */
			public static double getTimeout(String actionClassName) {
				try {
					java.lang.reflect.Field field = ActionTimeouts.class.getField(actionClassName);
					return field.getDouble(null);
				} catch (NoSuchFieldException | IllegalAccessException e) {
					// Action type not configured, return 0 (no timeout)
					return 0.0;
				}
			}
		}
	}
}
