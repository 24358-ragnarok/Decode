package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

public class HorizontalLauncher extends Mechanism {
	private final TrajectoryEngine trajectoryEngine;
	private final Servo horizontalServo;
	private final Servo verticalServo;
	private final SyncBelt belt;
	private final MatchSettings matchSettings;
	
	public HorizontalLauncher(
			DcMotor beltRight,
			DcMotor beltLeft,
			Servo horizontalServo,
			Servo verticalServo,
			TrajectoryEngine trajectoryEngine, MatchSettings matchSettings) {
		this.trajectoryEngine = trajectoryEngine;
		this.horizontalServo = horizontalServo;
		this.verticalServo = verticalServo;
		this.belt = new SyncBelt(beltRight, beltLeft);
		this.matchSettings = matchSettings;
	}
	
	/**
	 * Static helper method for testing pitch control without full launcher setup.
	 * Sets the pitch angle directly on a servo using the launcher's conversion
	 * logic.
	 */
	public static void setPitchDirect(Servo servo, double pitchDegrees) {
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
			return Settings.Launcher.DEFAULT_PITCH; // Default launch angle
		}
	}
	
	/**
	 * Aims the launcher at the target using feedback from the TrajectoryEngine.
	 * This is invoked by the {@link HorizontalLauncher#ready()} method which should
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
		belt.setTargetSpeed(requiredRPM);
		
		// Apply yaw correction: horizontalOffsetDegrees is the offset from center
		// A positive value means target is right, negative means left
		// We apply a proportional gain to smooth the correction
		if (Settings.Launcher.CORRECT_YAW) {
			double yawError = solution.horizontalOffsetDegrees;
			// Apply correction by adjusting current yaw position
			double currentYaw = getYaw();
			double targetYaw = currentYaw + yawError;
			setYaw(targetYaw);
		}
		
		// Set pitch directly: verticalOffsetDegrees is the absolute launch angle
		// No proportional correction needed - just set it
		setPitch(getPitch() + solution.verticalOffsetDegrees);
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
		
		// Check yaw alignment (horizontal offset from camera)
		boolean yawAligned = Math.abs(solution.horizontalOffsetDegrees) < Settings.Aiming.MAX_YAW_ERROR;
		
		// Check pitch alignment (servo has reached target angle)
		double pitchError = Math.abs(currentPitch - solution.verticalOffsetDegrees);
		boolean pitchAligned = pitchError < Settings.Aiming.MAX_PITCH_ERROR;
		
		// Check belt speed
		boolean beltReady = belt.atSpeed();
		
		return yawAligned && pitchAligned && beltReady;
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
		belt.spinUp();
		aim();
	}
	
	/**
	 * Maintains the launcher in a ready state without controlling the transfer.
	 * This keeps the belt spinning and maintains aim, but leaves the transfer
	 * free to be controlled by other commands (e.g., sequential firing).
	 * <p>
	 * Use this in loops when the transfer is being controlled separately.
	 */
	public void maintainReady() {
		belt.spinUp(); // spinUp() is safe to call repeatedly - it checks internally
		aim();
	}
	
	public void stop() {
		belt.spinDown();
		setPitch(Settings.Launcher.DEFAULT_PITCH);
	}
	
	/**
	 * Gets the current yaw angle in degrees.
	 * Yaw is centered at 0° for offset-based aiming.
	 * Range: [YAW_MIN_ANGLE, YAW_MAX_ANGLE] (e.g., -10° to +10°).
	 */
	public double getYaw() {
		if (Settings.Launcher.CORRECT_YAW) {
			return Settings.Launcher.servoToYaw(horizontalServo.getPosition());
		} else {
			return 0; // Center position
		}
	}
	
	/**
	 * Sets the yaw angle in degrees.
	 * Yaw is centered at 0° for offset-based aiming.
	 * Range: [YAW_MIN_ANGLE, YAW_MAX_ANGLE] (e.g., -10° to +10°).
	 * Clamping is handled by the conversion function.
	 */
	public void setYaw(double yawDegrees) {
		if (Settings.Launcher.CORRECT_YAW) {
			horizontalServo.setPosition(Settings.Launcher.yawToServo(yawDegrees));
		}
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
			return Settings.Launcher.DEFAULT_PITCH; // Default launch angle
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
	
	public final void init() {
		setYaw(0); // 0° yaw (center/forward)
		setPitch(Settings.Launcher.DEFAULT_PITCH);
	}
	
	public final void update() {
		belt.update();
	}
	
	/**
	 * A synchronous combination of two motors that maintains equal speeds using
	 * a proportional feedback controller.
	 * Supports variable target speeds for physics-based aiming.
	 * <p>
	 * TODO: Implement differential speeds to control topspin/backspin on launched
	 * artifacts
	 */
	public static class SyncBelt {
		private final DcMotor beltRight;
		private final DcMotor beltLeft;
		
		private long spinupTimestamp = 0;
		private boolean active = false;
		private double targetSpeedRPM = Settings.Aiming.DEFAULT_WHEEL_SPEED_RPM;
		
		// For smoothing
		private int lastRightPos = 0;
		private int lastLeftPos = 0;
		
		public SyncBelt(DcMotor right, DcMotor left) {
			this.beltRight = right;
			this.beltLeft = left;
			left.setDirection(DcMotor.Direction.REVERSE);
			right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		
		/**
		 * Sets the target wheel speed in RPM.
		 * This is used by complex aiming to achieve the calculated launch velocity.
		 */
		public void setTargetSpeed(double rpm) {
			// Clamp to safe range
			this.targetSpeedRPM = Math.max(Settings.Aiming.MIN_WHEEL_SPEED_RPM,
					Math.min(Settings.Aiming.MAX_WHEEL_SPEED_RPM, rpm));
			// If we're already spinning, adjust immediately
			if (active) {
				// Restart spinup timer if speed changed significantly
				spinupTimestamp = System.currentTimeMillis();
			}
		}
		
		/**
		 * Calculates motor power from target RPM.
		 * This is a simple linear mapping that should be calibrated for your motors.
		 */
		private double rpmToPower(double rpm) {
			// Simple linear mapping - adjust based on motor characteristics
			// Assumes max motor RPM at full power is around 6000 RPM
			double maxMotorRPM = 6000.0;
			double power = rpm / maxMotorRPM;
			
			// Clamp to valid motor power range
			return Math.max(0.0, Math.min(1.0, power));
		}
		
		public final void spinUp(double motorSpeed) {
			if (active)
				return;
			active = true;
			spinupTimestamp = System.currentTimeMillis();
			setBasePower(motorSpeed);
		}
		
		public final void spinUp() {
			if (!active) {
				active = true;
				spinupTimestamp = System.currentTimeMillis();
			}
			// Use target speed instead of fixed speed
			double targetPower = rpmToPower(targetSpeedRPM);
			setBasePower(targetPower);
		}
		
		public final void spinDown() {
			active = false;
			spinupTimestamp = 0;
			beltRight.setPower(0);
			beltLeft.setPower(0);
		}
		
		public boolean atSpeed() {
			return active &&
					System.currentTimeMillis() - spinupTimestamp > Settings.Launcher.BELT_SPINUP_TIME_MS;
		}
		
		/**
		 * Continuously try to match the actual speeds of the motors so they have the
		 * same tangential speed.
		 */
		public void update() {
			if (!active)
				return;
			
			int rightPos = beltRight.getCurrentPosition();
			int leftPos = beltLeft.getCurrentPosition();
			
			int deltaRight = rightPos - lastRightPos;
			int deltaLeft = leftPos - lastLeftPos;
			
			lastRightPos = rightPos;
			lastLeftPos = leftPos;
			
			// Prevent div-by-zero
			if (deltaRight == 0 && deltaLeft == 0)
				return;
			
			// Compute imbalance ratio
			double avg = (Math.abs(deltaRight) + Math.abs(deltaLeft)) / 2.0;
			double error = (deltaRight - deltaLeft) / avg;
			
			// Proportional correction
			double correction = Settings.Launcher.BELT_SYNC_KP * error;
			
			// Use target speed instead of fixed speed
			double base = rpmToPower(targetSpeedRPM);
			beltRight.setPower(base * (1 - correction));
			beltLeft.setPower(base * (1 + correction));
		}
		
		private void setBasePower(double power) {
			beltRight.setPower(power);
			beltLeft.setPower(power);
		}
	}
}