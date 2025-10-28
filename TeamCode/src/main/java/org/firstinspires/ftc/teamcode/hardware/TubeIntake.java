package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

@Deprecated
public class TubeIntake extends Mechanism {
	public final DcMotor intakeMotor;
	
	public final ColorSensor colorSensor;
	public final Servo[] servoArray;
	
	
	public MatchSettings.ArtifactColor expectation;
	
	public TubeIntake(DcMotor intakeMotor, Servo[] tubeServoArray, ColorSensor intakeColorSensor) {
		this.intakeMotor = intakeMotor;
		this.servoArray = tubeServoArray;
		this.colorSensor = intakeColorSensor;
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
		colorSensor.init();
	}
	
	public void update() {
		MatchSettings.ArtifactColor artifact = colorSensor.getArtifactColor();
		if (artifact == expectation || expectation == MatchSettings.ArtifactColor.UNKNOWN) {
			// if there is no expectation or it matches, let it through and reset expectation
			this.expectation = MatchSettings.ArtifactColor.UNKNOWN;
		} else {
			// the artifact is not the one we want, so spit it out
			out();
		}
	}
	
	public void setNextExpectedBall(MatchSettings.ArtifactColor expectation) {
		this.expectation = expectation;
	}
}