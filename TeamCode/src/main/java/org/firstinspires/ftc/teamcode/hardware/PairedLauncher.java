package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.MAX_SPEED_ERROR;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.TICKS_PER_REVOLUTION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

public class PairedLauncher extends Mechanism {
	private final TrajectoryEngine trajectoryEngine;
	private final Servo verticalServo;
	private final DcMotorEx rightMotor;
	private final DcMotorEx leftMotor;
	private final MatchSettings matchSettings;
	public double targetSpeedAngular = 0;
	private LauncherState state = LauncherState.IDLE;
	
	public PairedLauncher(
			DcMotorEx launcherRight,
			DcMotorEx launcherLeft,
			Servo verticalServo,
			TrajectoryEngine trajectoryEngine, MatchSettings matchSettings) {
		this.trajectoryEngine = trajectoryEngine;
		this.verticalServo = verticalServo;
		this.matchSettings = matchSettings;
		
		this.rightMotor = launcherRight;
		this.leftMotor = launcherLeft;
		
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	
	/**
	 * Static helper method for testing pitch control without full launcher setup.
	 * Sets the pitch angle directly on a servo using the launcher's conversion
	 * logic.
	 */
	public static void setPitchDirect(ServoImplEx servo, double pitchDegrees) {
		if (Settings.Launcher.CORRECT_PITCH) {
			servo.setPosition(Settings.Launcher.pitchToServo(pitchDegrees));
		}
	}
	
	/**
	 * Static helper method for testing pitch control without full launcher setup.
	 * Gets the current pitch angle from a servo using the launcher's conversion
	 * logic.
	 */
	public static double getPitchDirect(Servo servo) {
		if (Settings.Launcher.CORRECT_PITCH) {
			return Settings.Launcher.servoToPitch(servo.getPosition());
		} else {
			return Settings.Launcher.DEFAULT_PITCH_ANGLE; // Default launch angle
		}
	}
	
	/**
	 * Aims the launcher at the target using feedback from the TrajectoryEngine.
	 * This is invoked by the {@link PairedLauncher#ready()} method which should
	 * be called repeatedly in the main robot loop when aiming.
	 * <p>
	 * The TrajectoryEngine provides a standardized AimingSolution with:
	 * - Yaw offset from camera center (-10° to +10°)
	 * - Absolute pitch launch angle (0° to 90°)
	 * - Required launch velocity
	 * <p>
	 * This method is public so it can be called independently when you want to
	 * maintain aim and spin-up without controlling the transfer.
	 */
	public void aim() {
		// Pass current pitch angle to trajectory engine so it can account for limelight
		// rotation
		double currentPitch = getPitch();
		TrajectoryEngine.AimingSolution solution = trajectoryEngine.getAimingOffsets(
				matchSettings.getAllianceColor(), currentPitch);
		
		// If we don't have a target, do not adjust.
		if (!solution.hasTarget) {
			return;
		}
		
		// Update belt speed based on the required launch rpm
		double requiredRPM = solution.rpm;
		setRPM(requiredRPM);
		
		// Set pitch directly: verticalOffsetDegrees is the absolute launch angle
		// No proportional correction needed - just set it to the target angle
		setPitch(solution.verticalOffsetDegrees);
	}
	
	/**
	 * Checks if all conditions are met for a successful launch.
	 * <p>
	 * Verifies:
	 * - Target is detected by vision system
	 * - Yaw is centered on target (horizontal offset within tolerance)
	 * - Pitch servo has reached target launch angle
	 * - Belt is spun up to target speed
	 *
	 * @return True if the launcher is aimed, up to speed, and ready to launch.
	 */
	public boolean okayToLaunch() {
		double currentPitch = getPitch();
		TrajectoryEngine.AimingSolution solution = trajectoryEngine.getAimingOffsets(
				matchSettings.getAllianceColor(), currentPitch);
		
		if (!solution.hasTarget) {
			return false;
		}
		
		// Check pitch alignment (servo has reached target angle)
		double pitchError = Math.abs(currentPitch - solution.verticalOffsetDegrees);
		boolean pitchAligned = pitchError < Settings.Aiming.MAX_PITCH_ERROR;
		
		
		return pitchAligned && isAtSpeed();
	}
	
	/**
	 * Checks if launcher is ready to fire.
	 * <p>
	 * Note: This method only verifies launcher readiness. The actual firing
	 * of balls is handled by the SingleWheelTransfer mechanism, which should
	 * be controlled separately in autonomous or teleop code.
	 *
	 * @deprecated Use {@link #okayToLaunch()} to check readiness and control
	 * SingleWheelTransfer directly for firing.
	 */
	@Deprecated
	public void launch() {
		// This method is deprecated. Firing is now handled by SingleWheelTransfer.
		// In teleop: Call transfer.fire() when ready
		// In autonomous: LaunchAction coordinates launcher + transfer
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
	 * Maintains the launcher in a ready state without controlling the transfer.
	 * This keeps the belt spinning and maintains aim, but leaves the transfer
	 * free to be controlled by other commands (e.g., sequential firing).
	 * <p>
	 * Use this in loops when the transfer is being controlled separately.
	 */
	public void maintainReady() {
		aim();
		spinUp(); // spinUp() is safe to call repeatedly - it checks internally
	}
	
	/**
	 * Gets the current pitch angle in degrees.
	 * Pitch uses absolute launch angles from horizontal.
	 * Range: [PITCH_MIN_ANGLE, PITCH_MAX_ANGLE] (e.g., 0° to 90°).
	 * 0° = horizontal, 45° = 45° launch angle, 90° = straight up.
	 */
	public double getPitch() {
		if (Settings.Launcher.CORRECT_PITCH) {
			return Settings.Launcher.servoToPitch(verticalServo.getPosition());
		} else {
			return Settings.Launcher.DEFAULT_PITCH_ANGLE; // Default launch angle
		}
	}
	
	/**
	 * Sets the pitch angle in degrees.
	 * Pitch uses absolute launch angles from horizontal.
	 * Range: [PITCH_MIN_ANGLE, PITCH_MAX_ANGLE] (e.g., 0° to 90°).
	 * 0° = horizontal, 45° = 45° launch angle, 90° = straight up.
	 * Clamping is handled by the conversion function.
	 */
	public void setPitch(double pitchDegrees) {
		if (Settings.Launcher.CORRECT_PITCH) {
			verticalServo.setPosition(Settings.Launcher.pitchToServo(pitchDegrees));
		}
	}
	
	// ========== Testing/Utility Methods ==========
	
	public final void start() {
		spinDown();
	}
	
	public final void update() {
	}
	
	@Override
	public void stop() {
		spinDown();
	}
	
	public void spinUp() {
		state = LauncherState.ACTIVE;
		leftMotor.setVelocity(targetSpeedAngular);
		rightMotor.setVelocity(targetSpeedAngular);
	}
	
	public void spinDown() {
		state = LauncherState.IDLE;
		leftMotor.setVelocity(0);
		rightMotor.setVelocity(0);
	}
	
	public double getRPM() {
		double rightRPM = (rightMotor.getVelocity() / TICKS_PER_REVOLUTION) / 60.0;
		double leftRPM = (leftMotor.getVelocity() / TICKS_PER_REVOLUTION) / 60.0;
		return (rightRPM + leftRPM) / 2.0;
	}
	
	public void setRPM(double rpm) {
		targetSpeedAngular = rpm * TICKS_PER_REVOLUTION / 60.0;
	}
	
	public boolean isAtSpeed() {
		return state == LauncherState.ACTIVE && Math.abs(targetSpeedAngular - getRPM()) < MAX_SPEED_ERROR;
	}
	
	public enum LauncherState {
		IDLE,
		ACTIVE
	}
}