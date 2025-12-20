package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.ColorRangefinder;
import org.firstinspires.ftc.teamcode.software.ColorUnifier;
import org.firstinspires.ftc.teamcode.software.LimelightManager;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
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
	private static final LynxModule.BlinkerPolicy HUB_BLINKER_POLICY = new CustomBlinkerPolicy();
	public final Drivetrain drivetrain;
	public final Mechanism[] mechanismArray;
	// Optional non-mechanism helpers
	public final LimelightManager limelightManager;
	public final TrajectoryEngine trajectoryEngine;
	public final HardwareMap hardwareMap;
	private final List<LynxModule> allHubs;

	public MechanismManager(HardwareMap hw) {
		hardwareMap = hw;
		allHubs = hardwareMap.getAll(LynxModule.class);
		LynxModule.blinkerPolicy = HUB_BLINKER_POLICY;
		drivetrain = new Drivetrain(hw);

		// Build mechanisms safely
		FlexVectorIntake intake = createIntake();
		VerticalWheelTransfer transfer = createTransfer();
		LimelightManager ll = createLimelight();
		TrajectoryEngine traj = createTrajectory();
		PairedLauncher launcher = createLauncher();
		DualBallCompartment dbc = createCompartment();
		mechanismArray = new Mechanism[]{intake, transfer, launcher, dbc};

		// Save helpers
		limelightManager = ll;
		trajectoryEngine = traj;

		// Now that we've built all of the systems, begin caching system reads for
		// efficiency
		for (LynxModule hub : allHubs) {
			createHub(hub);
		}
	}

	private void createHub(LynxModule hub) {
		hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		hub.setPattern(HUB_BLINKER_POLICY.getIdlePattern(hub));
	}
	
	public void setHubColors(PresetColor c) {
		ArrayList<Blinker.Step> p = new ArrayList<>();
		int maxPatternLength = allHubs.get(0).getBlinkerPatternMaxLength();
		switch (c) {
			case RED:
				for (LynxModule hub : allHubs) {
					hub.setConstant(Color.RED);
				}
				break;
			case PURPLE:
				for (LynxModule hub : allHubs) {
					hub.setConstant(Color.MAGENTA);
				}
				break;
			case BLUE:
				for (LynxModule hub : allHubs) {
					hub.setConstant(Color.BLUE);
				}
				break;
			case RAINBOW:
			default:
				int targetLength = Math.max(1, maxPatternLength);
				
				int totalCycleTimeMs = 500;
				int stepDuration = totalCycleTimeMs / targetLength;
				
				for (int i = 0; i < targetLength; i++) {
					// Map the current step to a degree on the 360 degree color wheel
					float hue = (i * 360f) / targetLength;
					
					// Convert HSV to Android Color (Full Saturation and Value)
					int color = Color.HSVToColor(new float[]{hue, 1f, 1f});

					p.add(new Blinker.Step(color, stepDuration, TimeUnit.MILLISECONDS));
				}
				for (LynxModule hub : allHubs) {
					hub.setPattern(p);
				}
				break;
		}
	}
	
	private FlexVectorIntake createIntake() {
		if (!Settings.Deploy.INTAKE)
			return null;
		try {
			DcMotorEx intakeMotor = Settings.Hardware.INTAKE_MOTOR.fromHardwareMap(hardwareMap);
			// ColorRangefinder color1 = new ColorRangefinder(
			// hardwareMap, Settings.Hardware.COLOR_RANGEFINDER_1[0],
			// Settings.Hardware.COLOR_RANGEFINDER_1[1]);
			ColorRangefinder[] array = {};
			ColorUnifier color = new ColorUnifier(this, array);
			return new FlexVectorIntake(this, intakeMotor, color);
		} catch (Exception e) {
			return null;
		}
	}
	
	private VerticalWheelTransfer createTransfer() {
		if (!Settings.Deploy.TRANSFER)
			return null;
		
		DcMotorEx transferWheel = Settings.Hardware.TRANSFER_WHEEL_MOTOR.fromHardwareMap(hardwareMap);
		return new VerticalWheelTransfer(transferWheel);
	}
	
	private LimelightManager createLimelight() {
		if (!Settings.Deploy.LIMELIGHT)
			return null;
		
		return new LimelightManager(Settings.Hardware.LIMELIGHT.fromHardwareMap(hardwareMap));
	}
	
	private TrajectoryEngine createTrajectory() {
		if (!Settings.Deploy.TRAJECTORY_ENGINE)
			return null;
		try {
			return new TrajectoryEngine(this);
		} catch (Exception e) {
			return null;
		}
	}
	
	private PairedLauncher createLauncher() {
		if (!Settings.Deploy.LAUNCHER) {
			return null;
		}
		DcMotorEx right = Settings.Hardware.LAUNCHER_RIGHT.fromHardwareMap(hardwareMap);
		DcMotorEx left = Settings.Hardware.LAUNCHER_LEFT.fromHardwareMap(hardwareMap);
		ServoImplEx vertical = Settings.Hardware.LAUNCHER_PITCH_SERVO.fromHardwareMap(hardwareMap);
		ServoImplEx gate = Settings.Hardware.LAUNCHER_GATE.fromHardwareMap(hardwareMap);
		return new PairedLauncher(this, right, left, vertical, gate);
	}
	
	private DualBallCompartment createCompartment() {
		if (!Settings.Deploy.COMPARTMENT) {
			return null;
		}
		ServoImplEx right = Settings.Hardware.COMPARTMENT_RIGHT.fromHardwareMap(hardwareMap);
		ServoImplEx left = Settings.Hardware.COMPARTMENT_LEFT.fromHardwareMap(hardwareMap);
		return new DualBallCompartment(this, right, left);
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
		setHubColors(PresetColor.RAINBOW);
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
		setHubColors(PresetColor.RAINBOW);
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
	
	public enum PresetColor {
		RAINBOW,
		PURPLE,
		RED,
		BLUE,
	}
	
	private static class CustomBlinkerPolicy implements LynxModule.BlinkerPolicy {
		private static final int DEFAULT_CYCLE_MS = 2500;
		
		@Override
		public List<Blinker.Step> getIdlePattern(LynxModule lynxModule) {
			int maxSteps = Math.max(4, lynxModule.getBlinkerPatternMaxLength());
			int stepDuration = DEFAULT_CYCLE_MS / maxSteps;
			List<Blinker.Step> steps = new ArrayList<>();
			
			for (int i = 0; i < maxSteps; i++) {
				float hue = (i * 360f) / maxSteps;
				int color = Color.HSVToColor(new float[]{hue, 1f, 1f});
				steps.add(new Blinker.Step(color, stepDuration, TimeUnit.MILLISECONDS));
			}
			return steps;
		}
		
		@Override
		public List<Blinker.Step> getVisuallyIdentifyPattern(LynxModule lynxModule) {
			List<Blinker.Step> steps = new ArrayList<>();
			int stepDuration = 150;
			steps.add(new Blinker.Step(Color.WHITE, stepDuration, TimeUnit.MILLISECONDS));
			steps.add(new Blinker.Step(Color.BLACK, stepDuration, TimeUnit.MILLISECONDS));
			return steps;
		}
	}
}
