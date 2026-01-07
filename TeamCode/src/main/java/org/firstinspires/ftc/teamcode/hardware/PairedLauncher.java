package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.DEFAULT_PITCH_ANGLE;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.GATE_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.GATE_FIRE_POSITION;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.MAX_SPEED_ERROR;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.VELOCITY_ALPHA;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.rpmToTicksPerSec;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.ticksPerSecToRPM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

public class PairedLauncher extends Mechanism {
	private final ServoImplEx verticalServo;
	private final DcMotorEx rightMotor;
	private final DcMotorEx leftMotor;
	private final TimelockedServo gateServo;
	private final MechanismManager mechanisms;
	public double targetTPS = 0;
	private LauncherState state = LauncherState.IDLE;
	// Time-averaged velocity readings (exponential moving average)
	private double averagedRightRPM = 0;
	private double averagedLeftRPM = 0;
	
	public PairedLauncher(
			MechanismManager mechanisms,
			DcMotorEx launcherRight,
			DcMotorEx launcherLeft,
			ServoImplEx verticalServo, TimelockedServo gate) {
		this.mechanisms = mechanisms;
		this.verticalServo = verticalServo;
		
		this.rightMotor = launcherRight;
		this.leftMotor = launcherLeft;
		this.gateServo = gate;
		
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
	
	/**
	 * Aims the launcher at the target using feedback from the TrajectoryEngine.
	 * This is invoked by the {@link PairedLauncher#ready()} method which should
	 * be called repeatedly in the main robot loop when aiming.
	 * <p>
	 * The TrajectoryEngine provides a standardized AimingSolution with:
	 * - Absolute pitch launch angle (0° to 90°)
	 * - Required launch velocity
	 * <p>
	 * This method is public so it can be called independently when you want to
	 * maintain aim and spin-up without controlling the transfer.
	 */
	private void aim() {
		// Pass current pitch angle to trajectory engine so it can account for limelight
		// rotation
		TrajectoryEngine.AimingSolution solution = mechanisms.trajectoryEngine.getAimingOffsets(
				MatchState.getAllianceColor(), getPitch());
		
		if (!solution.hasTarget) {
			return;
		}
		
		setRPM(solution.rpm);
		setPitch(solution.verticalOffsetDegrees);
	}
	
	public void open() {
		gateServo.setPosition(GATE_FIRE_POSITION);
	}
	
	public void close() {
		gateServo.setPosition(GATE_CLOSED_POSITION);
	}
	
	public boolean isBusy() {
		return gateServo != null && gateServo.isBusy();
	}
	
	/**
	 * Readies the launcher to fire.
	 * This spins up the belt and aims at the target.
	 * Call this when you want to prepare for a single shot.
	 * <p>
	 * Note: This only prepares the launcher mechanism. The transfer must be
	 * controlled separately to actually fire balls.
	 */
	public void ready() {
		aim();
		spinUp();
	}
	
	/**
	 * Gets the current pitch angle in degrees.
	 * Pitch uses absolute launch angles from horizontal.
	 * Range: [PITCH_MIN_ANGLE, PITCH_MAX_ANGLE] (e.g., 0° to 90°).
	 * 0° = horizontal, 45° = 45° launch angle, 90° = straight up.
	 */
	public double getPitch() {
		return Settings.Launcher.servoToPitch(verticalServo.getPosition());
	}
	
	/**
	 * Sets the pitch angle in degrees.
	 * Pitch uses absolute launch angles from horizontal.
	 * Range: [PITCH_MIN_ANGLE, PITCH_MAX_ANGLE] (e.g., 0° to 90°).
	 * 0° = horizontal, 45° = 45° launch angle, 90° = straight up.
	 * Clamping is handled by the conversion function.
	 */
	public void setPitch(double pitchDegrees) {
		verticalServo.setPosition(Settings.Launcher.pitchToServo(pitchDegrees));
	}
	
	// ========== Testing/Utility Methods ==========
	
	public final void start() {
		spinDown();
		setPitch(DEFAULT_PITCH_ANGLE);
		close();
	}
	
	public final void update() {
		// Update time-averaged velocity readings using exponential moving average
		// This smooths out noise from instantaneous velocity readings
		double currentRightRPM = ticksPerSecToRPM(rightMotor.getVelocity());
		double currentLeftRPM = ticksPerSecToRPM(leftMotor.getVelocity());
		
		// Apply exponential moving average: new = alpha * current + (1 - alpha) * old
		// This naturally handles initialization (if averaged is 0, it will gradually
		// build up)
		averagedRightRPM = VELOCITY_ALPHA * currentRightRPM + (1 - VELOCITY_ALPHA) * averagedRightRPM;
		averagedLeftRPM = VELOCITY_ALPHA * currentLeftRPM + (1 - VELOCITY_ALPHA) * averagedLeftRPM;
		
		if (averagedLeftRPM < 0) {
			averagedLeftRPM = 0;
		}
		if (averagedRightRPM < 0) {
			averagedRightRPM = 0;
		}
	}
	
	@Override
	public void stop() {
		spinDown();
	}
	
	public void spinUp() {
		state = LauncherState.ACTIVE;
		leftMotor.setVelocity(targetTPS);
		rightMotor.setVelocity(targetTPS);
	}
	
	public void spinDown() {
		state = LauncherState.IDLE;
		leftMotor.setVelocity(0);
		rightMotor.setVelocity(0);
	}
	
	/**
	 * Gets the current RPM using time-averaged readings from both motors.
	 * Returns the average of the two motors' smoothed velocity readings.
	 */
	public double getRPM() {
		return (averagedRightRPM + averagedLeftRPM) / 2.0;
	}
	
	public void setRPM(double rpm) {
		targetTPS = rpmToTicksPerSec(rpm);
	}
	
	/**
	 * Checks if the launcher is at the target speed.
	 * Uses time-averaged velocity readings and checks both motors separately
	 * to ensure both are within tolerance, preventing false positives from
	 * averaging out errors between motors.
	 */
	public boolean isAtSpeed() {
		if (state != LauncherState.ACTIVE) {
			return false;
		}
		
		double targetRPM = ticksPerSecToRPM(targetTPS);
		
		// Check both motors separately - both must be within tolerance
		double rightError = Math.abs(targetRPM - averagedRightRPM);
		double leftError = Math.abs(targetRPM - averagedLeftRPM);
		
		return rightError < MAX_SPEED_ERROR && leftError < MAX_SPEED_ERROR;
	}
	
	public enum LauncherState {
		IDLE,
		ACTIVE
	}
}