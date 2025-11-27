package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.FIRING_POSITION_TICKS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.software.game.Artifact;

import java.util.Arrays;

/**
 * Transfer subsystem with positional awareness and CR wheel management.
 * <p>
 * Positions: index 0 = entrance (color sensor), index MAX_CAPACITY-1 = exit
 * (kicker).
 * <p>
 * Three CR wheels work in tandem:
 * - Main transfer wheel: moves balls through the slots
 * - Entrance wheel: spins to let balls in at the color sensor, holds closed
 * otherwise
 * - Exit wheel: spins to fire balls out, holds closed otherwise
 * <p>
 * Detection places a ball into the first available entrance slot and opens the
 * entrance
 * wheel briefly to allow entry. Advances shift slots toward the exit. Detection
 * uses a
 * blind window to avoid duplicate reads.
 * <p>
 * Automatic advance feature: The system automatically moves balls forward to
 * keep
 * one ready at the exit position when possible. This happens without
 * interfering
 * with intake operations - if a new ball is detected, it will be pulled in
 * before
 * continuing the advance. A grace period after detection ensures new balls have
 * time to enter the system.
 */
public class VerticalWheelTransfer extends Mechanism {
	private final DcMotorEx motor;
	public Artifact[] artifacts;
	private int targetTicks;
	
	public VerticalWheelTransfer(DcMotorEx motor) {
		this.motor = motor;
		motor.setPower(SPEED);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		artifacts = new Artifact[]{new Artifact(), new Artifact(), new Artifact()};
		this.targetTicks = motor.getCurrentPosition();
	}
	
	public void setUpForAuto() {
		artifacts = new Artifact[]{
				new Artifact(Artifact.Color.PURPLE, motor.getCurrentPosition()),
				new Artifact(Artifact.Color.PURPLE, motor.getCurrentPosition() + FIRING_POSITION_TICKS / 2),
				new Artifact(Artifact.Color.PURPLE, motor.getCurrentPosition() + FIRING_POSITION_TICKS)
		};
	}
	
	@Override
	public void start() {
	}
	
	@Override
	public void update() {
		motor.setTargetPosition(targetTicks);
	}
	
	@Override
	public void stop() {
		freeze();
	}
	
	/**
	 * Request a single-step advance (move everything one position toward exit).
	 * If the wheel is currently running this will instead increment pendingShifts
	 * and extend the run time accordingly (non-destructive).
	 */
	public void advance() {
		targetTicks += 100; // TODO convert ticks to rotation and use that as an input
	}
	
	public void reverse() {
		targetTicks -= 100;
	}
	
	public void move(int ticks) {
		targetTicks += ticks;
	}
	
	public void freeze() {
		targetTicks = motor.getCurrentPosition();
	}
	
	public void artifactIncoming(Artifact artifact) {
		for (int i = 0; i < artifacts.length - 1; i++) {
			if (artifacts[i].color == Artifact.Color.NONE) {
				artifacts[i] = artifact;
				advance();
			}
		}
	}
	
	public void moveNextArtifactToLauncher() {
		// find the artifact who's closest to the end, and then move the motor so that it gets to the end
		double greatest = Double.NaN;
		int greatestTicks = 0;
		for (int i = 0; i < artifacts.length - 1; i++) {
			if (artifacts[i].color == Artifact.Color.NONE) {
				continue;
			}
			int ticksTraveled = motor.getCurrentPosition() - artifacts[i].transferTicksWhenAtEntrance;
			if (Math.abs(ticksTraveled) > greatestTicks) {
				greatestTicks = Math.abs(ticksTraveled);
			}
		}
		move(FIRING_POSITION_TICKS - greatestTicks);
	}
	
	public boolean artifactInFiringPosition() {
		// find any artifact at or past firing position
		for (int i = 0; i < artifacts.length - 1; i++) {
			if (artifacts[i].color == Artifact.Color.NONE) {
				continue;
			}
			int ticksTraveled = motor.getCurrentPosition() - artifacts[i].transferTicksWhenAtEntrance;
			if (ticksTraveled > FIRING_POSITION_TICKS) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Returns a copy of current slots (positional snapshot).
	 */
	public Artifact[] getArtifactSnapshot() {
		return Arrays.copyOf(artifacts, artifacts.length);
	}
	
	public boolean isEmpty() {
		for (Artifact a : artifacts) {
			if (a.color != Artifact.Color.NONE) {
				return false;
			}
		}
		return true;
	}
}
