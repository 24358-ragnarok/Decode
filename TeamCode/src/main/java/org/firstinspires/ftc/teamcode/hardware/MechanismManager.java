package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.AlignmentEngine;
import org.firstinspires.ftc.teamcode.software.Drivetrain;
import org.firstinspires.ftc.teamcode.software.LimelightManager;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

import java.lang.reflect.Proxy;

public class MechanismManager {
	public final Drivetrain drivetrain;
	public final Mechanism[] mechanisms;
	
	// Optional non-mechanism helpers
	public final LimelightManager limelightManager;
	public final TrajectoryEngine trajectoryEngine;
	public final AlignmentEngine alignmentEngine;
	
	public MechanismManager(HardwareMap hw, MatchSettings match) {
		drivetrain = new Drivetrain(hw, match);
		
		// Build mechanisms safely
		Intake intake = createIntake(hw);
		Spindex spindex = createSpindex(hw, match);
		LimelightManager ll = createLimelight(hw, match);
		TrajectoryEngine traj = createTrajectory(ll, match);
		AlignmentEngine align = createAlignment(match, drivetrain);
		HorizontalLauncher launcher = createLauncher(hw, traj, match);
		
		mechanisms = new Mechanism[]{intake, spindex, launcher};
		
		// Save helpers
		limelightManager = ll;
		trajectoryEngine = traj;
		alignmentEngine = align;
	}
	
	private Intake createIntake(HardwareMap hw) {
		if (!Settings.Deploy.INTAKE)
			return null;
		try {
			ColorSensor sensor = new ColorSensor(hw.get(RevColorSensorV3.class, Settings.HardwareIDs.COLOR_SENSOR));
			DcMotor motor = hw.get(DcMotor.class, Settings.HardwareIDs.INTAKE_MOTOR);
			Servo[] servos = new Servo[4];
			for (int i = 0; i < 4; i++) {
				servos[i] = hw.get(Servo.class, Settings.HardwareIDs.INTAKE_SERVO_ARRAY[i]);
			}
			return new Intake(motor, servos, sensor);
		} catch (Exception e) {
			return null;
		}
	}
	
	private Spindex createSpindex(HardwareMap hw, MatchSettings match) {
		if (!Settings.Deploy.SPINDEX)
			return null;
		try {
			Servo spindexServo = hw.get(Servo.class, Settings.HardwareIDs.SPINDEX_SERVO);
			Servo launcherTransfer = hw.get(Servo.class, Settings.HardwareIDs.LAUNCHER_TRANSFER_SERVO);
			Servo intakeTransfer = hw.get(Servo.class, Settings.HardwareIDs.INTAKE_TRANSFER_SERVO);
			ColorSensor sensor = new ColorSensor(
					hw.get(RevColorSensorV3.class, Settings.HardwareIDs.SPINDEX_COLOR_SENSOR));
			return new Spindex(spindexServo, launcherTransfer, intakeTransfer, sensor, match);
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
	
	private AlignmentEngine createAlignment(MatchSettings match, Drivetrain dt) {
		if (!Settings.Deploy.ALIGNMENT_ENGINE)
			return null;
		try {
			return new AlignmentEngine(match, dt);
		} catch (Exception e) {
			return null;
		}
	}
	
	private HorizontalLauncher createLauncher(HardwareMap hw, TrajectoryEngine traj, MatchSettings matchSettings) {
		if (!Settings.Deploy.LAUNCHER)
			return null;
		try {
			DcMotor right = hw.get(DcMotor.class, Settings.HardwareIDs.LAUNCHER_RIGHT);
			DcMotor left = hw.get(DcMotor.class, Settings.HardwareIDs.LAUNCHER_LEFT);
			Servo horizontal;
			if (Settings.Launcher.CORRECT_YAW) {
				horizontal = hw.get(Servo.class, Settings.HardwareIDs.LAUNCHER_YAW_SERVO);
			} else {
				// make a dummy servo instead
				horizontal = dummyServo();
			}
			Servo vertical = hw.get(Servo.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
			return new HorizontalLauncher(right, left, horizontal, vertical, traj, matchSettings);
		} catch (Exception e) {
			return null;
		}
	}
	
	public Servo dummyServo() {
		//noinspection SuspiciousInvocationHandlerImplementation
		return (Servo) Proxy.newProxyInstance(
				Servo.class.getClassLoader(),
				new Class[]{Servo.class},
				(proxy, method, args) -> {
					return null; // swallow exceptions
				});
	}
	
	@SuppressWarnings("unchecked")
	public <T extends Mechanism> @Nullable T get(Class<T> type) {
		for (Mechanism m : mechanisms) {
			if (m != null && type.isInstance(m)) {
				return (T) m;
			}
		}
		return null;
	}
	
	public void init() {
		for (Mechanism m : mechanisms)
			if (m != null)
				m.init();
	}
	
	public void update() {
		for (Mechanism m : mechanisms)
			if (m != null)
				m.update();
		drivetrain.update();
	}
	
	public void stop() {
		for (Mechanism m : mechanisms)
			if (m != null)
				m.stop();
	}
}
