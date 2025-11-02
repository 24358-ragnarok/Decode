package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

public class FlywheelIntake extends Mechanism {
	private final DcMotor intakeMotor;
	public IntakeState state;
	
	
	public MatchSettings.ArtifactColor expectation;
	
	public FlywheelIntake(DcMotor intakeMotor) {
		this.intakeMotor = intakeMotor;
		this.state = IntakeState.STOPPED;
	}
	
	public void in() {
		state = IntakeState.IN;
		intakeMotor.setPower(SPEED);
	}
	
	public void out() {
		state = IntakeState.OUT;
		intakeMotor.setPower(-SPEED);
	}
	
	public void stop() {
		state = IntakeState.STOPPED;
		intakeMotor.setPower(0);
	}
	
	public void init() {
		in();
	}
	
	public void toggleIn() {
		if (state == IntakeState.IN) {
			stop();
		} else {
			in();
		}
	}
	
	public void toggleOut() {
		if (state == IntakeState.OUT) {
			stop();
		} else {
			out();
		}
	}
	
	public void update() {
	}
	
	public enum IntakeState {
		IN,
		OUT,
		STOPPED
	}
}