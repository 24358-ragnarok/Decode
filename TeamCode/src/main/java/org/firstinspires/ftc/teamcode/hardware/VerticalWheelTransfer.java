package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.CRAWL_SPEED;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.CRAWL_TICKS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.DECREMENT_TICKS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.INCREMENT_TICKS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.SPEED;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_MOTOR_SPINUP_MS;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.software.ColorUnifier;
import org.firstinspires.ftc.teamcode.software.game.Artifact;

/**
 * Transfer subsystem controlling the vertical wheel that moves balls.
 * <p>
 * Provides simple movement commands (advance, reverse, crawl) with position
 * tracking.
 * Ball detection and sorting logic is handled externally by SortedLaunchAction
 * using the ColorUnifier at the swap position.
 */
public class VerticalWheelTransfer extends Mechanism {
	public final ColorUnifier colorUnifier;
	private final DcMotorEx motor;
	public Timer debounce = new Timer();
	private double targetTicks;
	
	public VerticalWheelTransfer(DcMotorEx motor, ColorUnifier unifier) {
		this.motor = motor;
		this.colorUnifier = unifier;
		motor.setTargetPosition(motor.getCurrentPosition());
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(SPEED);
		motor.setDirection(DcMotorSimple.Direction.FORWARD);
		motor.setTargetPositionTolerance(POSITION_TOLERANCE);
		this.targetTicks = motor.getCurrentPosition();
	}

	@Override
	public void start() {
	}
	
	@Override
	public void update() {
		motor.setTargetPosition((int) targetTicks);
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
		move(INCREMENT_TICKS);
	}
	
	public void reverse() {
		move(DECREMENT_TICKS);
	}
	
	public void move(double ticks) {
		motor.setPower(SPEED);
		targetTicks += ticks;
		debounce.resetTimer();
	}
	
	public void freeze() {
		targetTicks = motor.getCurrentPosition();
	}
	
	public void crawl() {
		motor.setPower(CRAWL_SPEED);
		targetTicks += CRAWL_TICKS;
		debounce.resetTimer();
	}
	
	public boolean isBusy() {
		return motor.isBusy() && debounce.getElapsedTime() > TRANSFER_MOTOR_SPINUP_MS;
	}
	
	public int getTicks() {
		return motor.getCurrentPosition();
	}
	
	public Artifact detect() {
		return colorUnifier.find();
	}
}
