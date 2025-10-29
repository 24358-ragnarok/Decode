package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.SPEED;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

public class FlywheelIntake extends Mechanism {
	public final CRServo intakeServo;
	
	
	public MatchSettings.ArtifactColor expectation;
	
	public FlywheelIntake(CRServo intakeServo) {
		this.intakeServo = intakeServo;
		this.expectation = MatchSettings.ArtifactColor.UNKNOWN;
	}
	
	public void in() {
		intakeServo.setPower(SPEED);
	}
	
	public void out() {
		intakeServo.setPower(-SPEED);
	}
	
	public void stop() {
		intakeServo.setPower(0);
	}
	
	public void init() {
		intakeServo.setPower(0);
	}
	
	public void update() {
	}
}