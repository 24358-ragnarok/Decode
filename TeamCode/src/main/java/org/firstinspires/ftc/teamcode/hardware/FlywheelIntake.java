package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

public class FlywheelIntake extends Mechanism {
	public final DcMotor intakeMotor;
	
	
	public MatchSettings.ArtifactColor expectation;
	
	public FlywheelIntake(DcMotor intakeMotor) {
		this.intakeMotor = intakeMotor;
		this.expectation = MatchSettings.ArtifactColor.UNKNOWN;
	}
	
	public void in() {
		intakeMotor.setPower(SPEED);
	}
	
	public void out() {
		intakeMotor.setPower(-SPEED);
	}
	
	public void stop() {
		intakeMotor.setPower(0);
	}
	
	public void init() {
		intakeMotor.setPower(0);
	}
	
	public void update() {
	}
}