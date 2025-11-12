package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.ColorSensor;
import org.firstinspires.ftc.teamcode.software.LimelightManager;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

import java.lang.reflect.Proxy;
import java.util.List;
import java.util.function.Consumer;

/**
 * Central manager for all robot mechanisms and subsystems.
 * <p>
 * Handles safe initialization, lifecycle management, and dependency injection
 * for all mechanisms.
 * Uses a fail-safe approach where missing hardware components are gracefully
 * handled without
 * crashing the robot program.
 * <p>
 * Key features:
 * - Safe mechanism creation with exception handling
 * - Type-safe mechanism retrieval
 * - Centralized lifecycle management (init, update, stop)
 * - Automatic dependency resolution between mechanisms
 */
public class MechanismManager {
	public final Drivetrain drivetrain;
	public final Mechanism[] mechanismArray;

	// Optional non-mechanism helpers
	public final LimelightManager limelightManager;
	public final TrajectoryEngine trajectoryEngine;
	public final MatchSettings matchSettings;
	List<LynxModule> allHubs;
	

	public MechanismManager(HardwareMap hw, MatchSettings match) {
		allHubs = hardwareMap.getAll(LynxModule.class);
		drivetrain = new Drivetrain(hw, match);
		matchSettings = match;

		// Build mechanisms safely
		FlywheelIntake intake = createIntake(hw);
		SingleWheelTransfer transfer = createTransfer(hw, intake);
		LimelightManager ll = createLimelight(hw, match);
		TrajectoryEngine traj = createTrajectory(ll, match);
		HorizontalLauncher launcher = createLauncher(hw, traj, match);
		
		mechanismArray = new Mechanism[]{intake, transfer, launcher};

		// Save helpers
		limelightManager = ll;
		trajectoryEngine = traj;
		
		// Now that we've built all of the systems, begin caching system reads for efficiency
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
			hub.setConstant(Settings.Color.RAGNAROK_RED);
		}
	}

	private FlywheelIntake createIntake(HardwareMap hw) {
		if (!Settings.Deploy.INTAKE)
			return null;
		try {
			DcMotorEx intakeMotor = hw.get(DcMotorEx.class, Settings.HardwareIDs.INTAKE_MOTOR);
			return new FlywheelIntake(intakeMotor);
		} catch (Exception e) {
			return null;
		}
	}

	private SingleWheelTransfer createTransfer(HardwareMap hw, FlywheelIntake intake) {
		if (!Settings.Deploy.TRANSFER)
			return null;
		try {
			CRServo transferWheel = hw.get(CRServo.class, Settings.HardwareIDs.TRANSFER_WHEEL_SERVO);
			CRServo entranceWheel = hw.get(CRServo.class, Settings.HardwareIDs.TRANSFER_ENTRANCE_WHEEL);
			ServoImplEx exitWheel = hw.get(ServoImplEx.class, Settings.HardwareIDs.TRANSFER_EXIT_KICKER);
			RevColorSensorV3 sensor = hw.get(RevColorSensorV3.class, Settings.HardwareIDs.TRANSFER_COLOR_SENSOR);
			ColorSensor color = new ColorSensor(sensor);
			// ColorRangefinder color = new ColorRangefinder(hw);
			return new SingleWheelTransfer(transferWheel, entranceWheel, exitWheel, color, intake);
		} catch (Exception e) {
			return null;
		}
	}
	
	private LimelightManager createLimelight(HardwareMap hw, MatchSettings match) {
		if (!Settings.Deploy.LIMELIGHT)
			return null;
		try {
			return new LimelightManager(hw.get(Limelight3A.class, Settings.HardwareIDs.LIMELIGHT), match);
		} catch (Exception e) {
			return null;
		}
	}
	
	private TrajectoryEngine createTrajectory(LimelightManager ll, MatchSettings match) {
		if (!Settings.Deploy.TRAJECTORY_ENGINE)
			return null;
		try {
			return new TrajectoryEngine(ll, match, drivetrain);
		} catch (Exception e) {
			return null;
		}
	}
	
	private HorizontalLauncher createLauncher(HardwareMap hw, TrajectoryEngine traj, MatchSettings matchSettings) {
		if (!Settings.Deploy.LAUNCHER)
			return null;
		try {
			DcMotorEx right = hw.get(DcMotorEx.class, Settings.HardwareIDs.LAUNCHER_RIGHT);
			DcMotorEx left = hw.get(DcMotorEx.class, Settings.HardwareIDs.LAUNCHER_LEFT);
			ServoImplEx horizontal;
			if (Settings.Launcher.CORRECT_YAW) {
				horizontal = hw.get(ServoImplEx.class, Settings.HardwareIDs.LAUNCHER_YAW_SERVO);
			} else {
				// make a dummy servo instead
				horizontal = dummyServo();
			}
			ServoImplEx vertical = hw.get(ServoImplEx.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
			return new HorizontalLauncher(right, left, horizontal, vertical, traj, matchSettings);
		} catch (Exception e) {
			return null;
		}
	}
	
	/**
	 * Creates a dummy servo that swallows all method calls.
	 * Used when hardware is not available but code expects a servo instance.
	 *
	 * @return A proxy servo that does nothing
	 */
	public ServoImplEx dummyServo() {
		return (ServoImplEx) Proxy.newProxyInstance(
				ServoImplEx.class.getClassLoader(),
				new Class[]{ServoImplEx.class},
				(proxy, method, args) -> {
					return null; // swallow exceptions
				});
	}
	
	/**
	 * Retrieves a mechanism of the specified type.
	 * 
	 * @param type The class type of the mechanism to retrieve
	 * @return The mechanism instance, or null if not available
	 */
	@SuppressWarnings("unchecked")
	public <T extends Mechanism> @Nullable T get(Class<T> type) {
		for (Mechanism m : mechanismArray) {
			if (m != null && type.isInstance(m)) {
				return (T) m;
			}
		}
		return null;
	}
	
	/**
	 * Initializes all available mechanisms.
	 */
	public void start() {
		for (Mechanism m : mechanismArray) {
			if (m != null) {
				m.start();
			}
		}
	}
	
	/**
	 * Updates all available mechanisms and the drivetrain.
	 */
	public void update() {
		// prepare for new hardware reads
		for (LynxModule hub : allHubs) {
			hub.clearBulkCache();
		}
		
		for (Mechanism m : mechanismArray) {
			if (m != null) {
				m.update();
			}
		}
		drivetrain.update();
	}
	
	/**
	 * Stops all available mechanisms safely.
	 */
	public void stop() {
		for (Mechanism m : mechanismArray) {
			if (m != null) {
				m.stop();
			}
		}
	}
	
	/**
	 * Executes an action only if the specified object is not null.
	 * This allows safe execution of operations on mechanisms that may not be
	 * available.
	 *
	 * @param obj    The object to check (may be null)
	 * @param action The action to execute if the object is valid
	 * @param <T>    The type of the object
	 */
	public <T> void ifValid(T obj, Consumer<T> action) {
		if (obj != null) {
			action.accept(obj);
		}
	}
}
