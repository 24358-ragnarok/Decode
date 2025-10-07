package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.TrajectoryEngine;

public class Launcher extends Mechanism {
	private final TrajectoryEngine trajectoryEngine;
	private final Servo horizontalServo;
	private final Servo verticalServo;
	private final Spindex spindex;
	private final SyncBelt belt;
	
	public Launcher(Spindex spindex,
	                DcMotor beltRight,
	                DcMotor beltLeft,
	                Servo horizontalServo,
	                Servo verticalServo,
	                TrajectoryEngine trajectoryEngine) {
		this.spindex = spindex;
		this.trajectoryEngine = trajectoryEngine;
		this.horizontalServo = horizontalServo;
		this.verticalServo = verticalServo;
		this.belt = new SyncBelt(beltRight, beltLeft);
	}
	
	/**
	 * Aims the launcher at the target using feedback from the TrajectoryEngine.
	 * This is invoked by the {@link Launcher#ready()} method which should be called
	 * repeatedly in the main robot loop when aiming.
	 * <p>
	 * This method is public so it can be called independently when you want to
	 * maintain aim and spin-up without controlling the spindex.
	 */
	public void aim() {
		TrajectoryEngine.AimingOffsets offsets = trajectoryEngine.getAimingOffsets();
		
		// If we don't have a target, do not adjust.
		if (!offsets.hasTarget) {
			return;
		}
		
		// 1. Read the current physical orientation from the servos
		double currentYaw = getYaw();
		double currentPitch = getPitch();
		
		// 2. Calculate the correction needed. The error is the offset from the camera.
		// We add the error multiplied by a gain (Kp) to the current position.
		// For yaw, a positive offset (tx) means the target is to the right, so we
		// increase yaw.
		// For pitch, a positive offset (ty) means the target is up, so we increase
		// pitch.
		double yawCorrection = offsets.horizontalOffsetDegrees * Settings.Launcher.AIM_YAW_KP;
		double pitchCorrection = offsets.verticalOffsetDegrees * Settings.Launcher.AIM_PITCH_KP;
		
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
	
	/**
	 * Checks if all conditions are met for a successful launch.
	 *
	 * @return True if the launcher is aimed, up to speed, and a game piece is
	 * ready.
	 */
	public boolean okayToLaunch() {
		return trajectoryEngine.isAimed() &&
				belt.atSpeed() &&
				spindex.isNextArtifactAtExit();
	}
	
	/**
	 * Launches the artifact if possible.
	 */
	public void launch() {
		if (!okayToLaunch())
			return;
		
		spindex.eject();
	}
	
	/**
	 * Readies the launcher to fire.
	 * This spins up the belt, aims, and prepares the spindex.
	 * Call this when you want to prepare for a single shot.
	 */
	public void ready() {
		belt.spinUp();
		aim();
		spindex.rotateNextArtifactToExit();
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
	 * <p>
	 * TODO: Implement differential speeds to control topspin/backspin on launched
	 * artifacts
	 */
	public static class SyncBelt {
		private final DcMotor beltRight;
		private final DcMotor beltLeft;
		
		private long spinupTimestamp = 0;
		private boolean active = false;
		
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
		
		public final void spinUp(double motorSpeed) {
			if (active)
				return;
			active = true;
			spinupTimestamp = System.currentTimeMillis();
			setBasePower(motorSpeed);
		}
		
		public final void spinUp() {
			spinUp(Settings.Launcher.BELT_MOTOR_SPEED);
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
		 * same
		 * tangential speed.
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
			
			double base = Settings.Launcher.BELT_MOTOR_SPEED;
			beltRight.setPower(base * (1 - correction));
			beltLeft.setPower(base * (1 + correction));
		}
		
		private void setBasePower(double power) {
			beltRight.setPower(power);
			beltLeft.setPower(power);
		}
	}
}