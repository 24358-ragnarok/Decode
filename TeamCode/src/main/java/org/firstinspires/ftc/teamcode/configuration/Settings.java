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
		public static final String INTAKE_MOTOR = "intakeMotor";
		public static final String LAUNCHER_RIGHT = "launcherRight";
		public static final String LAUNCHER_LEFT = "launcherLeft";
		public static final String[] INTAKE_SERVO_ARRAY = {"intakeServoLowerLeft", "intakeServoLowerRight",
				"intakeServoUpperLeft", "intakeServoUpperRight"};
		public static final String LAUNCHER_TRANSFER_SERVO = "launcherTransferServo";
		public static final String INTAKE_TRANSFER_SERVO = "intakeTransferServo";
		public static final String SPINDEX_SERVO = "spindexServo";
		public static final String LAUNCHER_HORIZONTAL_SERVO = "launcherHorizontalServo";
		public static final String LAUNCHER_VERTICAL_SERVO = "launcherVerticalServo";
		
		// Sensors
		public static final String SPINDEX_COLOR_SENSOR = "spindexColorSensor";
		public static final String LIMELIGHT = "limelight";
		public static final String COLOR_SENSOR = "colorSensor";
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
	 * Settings for the Launcher mechanism.
	 */
	@Configurable
	public static class Launcher {
		public static double BELT_MOTOR_SPEED = 1.0; // Launcher motor speed (0..1)
		public static long BELT_SPINUP_TIME_MS = 500;
		public static double BELT_SYNC_KP = 0.05; // Proportional gain for synchronizing belt speeds
		public static double MIN_PITCH = 0; // degrees from horizontal when vert servo is min
		public static double MAX_PITCH = 30; // degrees from horizontal when vert servo is max
		public static double MIN_YAW = -20; // degrees left when horizontal servo is min
		public static double MAX_YAW = 20; // degrees right when horizontal servo is max
		
		public static double AIM_YAW_KP = 0.05; // Proportional gain for yaw correction
		public static double AIM_PITCH_KP = 0.05; // TODO tune; set up limelight launcher and increase these until
		// oscillation occurs
	}
	
	@Configurable
	public static class Vision {
		public static double LL_WINDOW_SIZE_DEGREES = 40; // Horizontal window size
	}
	
	@Configurable
	public static class ColorSensor {
		public static double[] GREEN_TARGET = {0, 200, 0};
		public static double[] PURPLE_TARGET = {200, 0, 200};
		public static double CONFIDENCE_THRESHOLD = 60.0; // Acceptable distance threshold
	}
	
	@Configurable
	public static class Aiming {
		public static final double GRAVITY = 9.81;
		public static double MUZZLE_TANGENTIAL_MAX_SPEED; // m/s, TODO: tune
		public static double MUZZLE_HEIGHT = 5; // inches, TODO: tune
		public static double GOAL_HEIGHT = 37.5; // inches
		/**
		 * Note that ROTATIONAL error refers to the chassis rotation relative to the
		 * goal.
		 * YAW refers to the launcher horizontal angle
		 * PITCH refers to the launcher vertical angle
		 */
		public static double MAX_ROTATIONAL_ERROR = Math.toRadians(10);
		public static double MAX_YAW_ERROR = Math.toRadians(5);
		public static double MAX_PITCH_ERROR = Math.toRadians(2);
	}
	
	/**
	 * Key locations on the game field.
	 */
	@Configurable
	public static class Field {
		public static double WIDTH = 144.0; // inches
		public static Pose RESET_POSE = new Pose(72, 72, Math.toRadians(270));
		public static Pose RED_GOAL_POSE = new Pose(131, 137.5, Math.toRadians(225));
		public static Pose BLUE_GOAL_POSE = new Pose(12.5, 137.5, Math.toRadians(315));
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
	 */
	@Configurable
	public static class Deploy {
		public static boolean INTAKE = false;
		public static boolean LAUNCHER = false;
		public static boolean LIMELIGHT = true;
		public static boolean SPINDEX = false;
		public static boolean TRAJECTORY_ENGINE = false;
		public static boolean ALIGNMENT_ENGINE = true;
	}
	
	// A static class to hold all pose constants for organization.
	public static class Autonomous {
		// NOTE: BLUE alliance paths are the reference (we have a blue field for
		// tuning).
		// RED alliance paths are automatically mirrored by the PathRegistry.
		// Headings are in radians. 90 degrees = Math.toRadians(90)
		
		// Poses for the FAR side of the field, BLUE alliance TODO (REFERENCE - tune these!)
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
		
		// Poses for the CLOSE side of the field, BLUE alliance TODO (REFERENCE - tune these!)
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
