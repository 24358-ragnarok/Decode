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
	 * Aims the launcher at the target using feedback from the TrajectoryEngine.
	 * This is invoked by the {@link HorizontalLauncher#ready()} method which should
	 * be called
	 * repeatedly in the main robot loop when aiming.
	 * <p>
	 * This method is public so it can be called independently when you want to
	 * maintain aim and spin-up without controlling the spindex.
	 */
	public void aim() {
		TrajectoryEngine.AimingSolution solution = trajectoryEngine.getAimingOffsets(matchSettings.getAllianceColor());
		
		// If we don't have a target, do not adjust.
		if (!solution.hasTarget) {
			return;
		}
		
		// Update belt speed based on the required launch velocity
		double requiredRPM = solution.getRequiredWheelSpeedRPM();
		belt.setTargetSpeed(requiredRPM);
		
		if (Settings.Aiming.USE_COMPLEX_AIMING) {
			// Complex aiming: directly set the calculated angles
			double targetYaw = Math.max(Settings.Launcher.MIN_YAW,
					Math.min(Settings.Launcher.MAX_YAW, solution.horizontalOffsetDegrees));
			double targetPitch = Math.max(Settings.Launcher.MIN_PITCH,
					Math.min(Settings.Launcher.MAX_PITCH, solution.verticalOffsetDegrees));
			
			setYaw(targetYaw);
			setPitch(targetPitch);
		} else {
			// Simple aiming: use proportional correction based on offsets
			// 1. Read the current physical orientation from the servos
			double currentYaw = getYaw();
			double currentPitch = getPitch();
			
			// 2. Calculate the correction needed. The error is the offset from the camera.
			// We add the error multiplied by a gain (Kp) to the current position.
			// For yaw, a positive offset (tx) means the target is to the right, so we
			// increase yaw.
			// For pitch, a positive offset (ty) means the target is up, so we increase
			// pitch.
			double yawCorrection = solution.horizontalOffsetDegrees * Settings.Launcher.AIM_YAW_KP;
			double pitchCorrection = solution.verticalOffsetDegrees * Settings.Launcher.AIM_PITCH_KP;
			
			// 3. Calculate the new target orientation and adjust
			// If the amount of correction is outside the acceptable range, adjust.
			if (Math.abs(yawCorrection) > Settings.Aiming.MAX_YAW_ERROR) {
				double targetYaw = Math.max(Settings.Launcher.MIN_YAW,
						Math.min(Settings.Launcher.MAX_YAW, currentYaw + yawCorrection));
				setYaw(targetYaw);
			}
			
			if (Math.abs(pitchCorrection) > Settings.Aiming.MAX_PITCH_ERROR) {
				double targetPitch = Math.max(Settings.Launcher.MIN_PITCH,
						Math.min(Settings.Launcher.MAX_PITCH, currentPitch + pitchCorrection));
				setPitch(targetPitch);
			}
		}
	}
	
	/**
	 * Checks if all conditions are met for a successful launch.
	 *
	 * @return True if the launcher is aimed, up to speed, and a game piece is
	 * ready.
	 */
	public boolean okayToLaunch() {
		return trajectoryEngine.isAimed() &&
				belt.atSpeed();
	}
	
	/**
	 * Launches the artifact if possible.
	 */
	public void launch() {
		if (!okayToLaunch()) {
		}
		
		// TODO kicker?
	}
	
	/**
	 * Readies the launcher to fire.
	 * This spins up the belt, aims, and prepares the spindex.
	 * Call this when you want to prepare for a single shot.
	 */
	public void ready() {
		belt.spinUp();
		aim();
	}
	
	/**
	 * Maintains the launcher in a ready state without controlling the spindex.
	 * This keeps the belt spinning and maintains aim, but leaves the spindex
	 * free to be controlled by other commands (e.g., rapid fire).
	 * <p>
	 * Use this in loops when the spindex is being controlled separately.
	 */
	public void maintainReady() {
		belt.spinUp(); // spinUp() is safe to call repeatedly - it checks internally
		aim();
	}
	
	public void stop() {
		belt.spinDown();
	}
	
	public double getYaw() {
		if (Settings.Launcher.CORRECT_YAW) {
			return servoToYaw(horizontalServo.getPosition());
		} else {
			return 0;
		}
	}
	
	public void setYaw(double yaw) {
		if (Settings.Launcher.CORRECT_YAW) {
			horizontalServo.setPosition(yawToServo(yaw));
		}
	}
	
	public double getPitch() {
		if (Settings.Launcher.CORRECT_PITCH) {
			return servoToPitch(verticalServo.getPosition());
		} else {
			return 0;
		}
	}
	
	public void setPitch(double pitch) {
		if (Settings.Launcher.CORRECT_PITCH) {
			verticalServo.setPosition(pitchToServo(pitch));
		}
	}
	
	private double yawToServo(double yawDegrees) {
		return (yawDegrees - Settings.Launcher.MIN_YAW) / (Settings.Launcher.MAX_YAW - Settings.Launcher.MIN_YAW);
	}
	
	private double pitchToServo(double pitchDegrees) {
		return (pitchDegrees - Settings.Launcher.MIN_PITCH)
				/ (Settings.Launcher.MAX_PITCH - Settings.Launcher.MIN_PITCH);
	}
	
	private double servoToYaw(double servoPos) {
		return Settings.Launcher.MIN_YAW + servoPos * (Settings.Launcher.MAX_YAW - Settings.Launcher.MIN_YAW);
	}
	
	private double servoToPitch(double servoPos) {
		return Settings.Launcher.MIN_PITCH + servoPos * (Settings.Launcher.MAX_PITCH - Settings.Launcher.MIN_PITCH);
	}
	
	public final void init() {
		setYaw(0);
		setPitch(0);
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
		private double targetSpeedRPM = Settings.Aiming.WHEEL_SPEED_RPM;
		
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