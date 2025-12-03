package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.BALL_TRAVEL_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.COLOR_DETECTION_DEBOUNCE_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Intake.SPEED;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.software.ColorSensor;

/**
 * FlywheelIntake controls the robot's intake mechanism using a flywheel motor.
 * Supports intake, outtake, and stop operations with state tracking.
 * <p>
 * Also manages color detection at the intake location. When intaking, polls the
 * color sensor with debounce and notifies the transfer system of detected balls
 * with a travel time delay to account for physical movement from intake to
 * transfer.
 */
public class FlywheelIntake extends Mechanism {
	private final DcMotorEx intakeMotor;
	private final ColorSensor colorSensor;
	public IntakeState state;
	private SingleWheelTransfer transfer; // Set by transfer during construction
	// Color detection state
	private long lastDetectionTimeMs = 0;
	private MatchSettings.ArtifactColor lastDetectedColor = MatchSettings.ArtifactColor.UNKNOWN;

	/**
	 * Creates a new FlywheelIntake instance.
	 *
	 * @param intakeMotor The DC motor that controls the intake flywheel
	 * @param colorSensor The color sensor at the intake location
	 */
	public FlywheelIntake(DcMotorEx intakeMotor, ColorSensor colorSensor) {
		this.intakeMotor = intakeMotor;
		this.colorSensor = colorSensor;
		this.state = IntakeState.STOPPED;
		if (colorSensor != null) {
			colorSensor.init();
		}
	}
	
	/**
	 * Sets the transfer mechanism reference for ball registration callbacks.
	 * Called by SingleWheelTransfer during construction.
	 */
	void setTransfer(SingleWheelTransfer transfer) {
		this.transfer = transfer;
	}

	/**
	 * Starts the intake motor to pull artifacts in.
	 */
	public void in() {
		state = IntakeState.IN;
		intakeMotor.setPower(SPEED);
	}

	/**
	 * Reverses the intake motor to push artifacts out.
	 */
	public void out() {
		state = IntakeState.OUT;
		intakeMotor.setPower(-SPEED);
	}
	
	/**
	 * Stops the intake motor.
	 */
	public void stop() {
		state = IntakeState.STOPPED;
		intakeMotor.setPower(0);
	}
	
	/**
	 * Initializes the intake by starting the motor.
	 */
	@Override
	public void start() {
		stop();
	}
	
	/**
	 * Toggles between intake and stopped states.
	 */
	public void toggleIn() {
		if (state == IntakeState.IN) {
			stop();
		} else {
			in();
		}
	}
	
	/**
	 * Toggles between outtake and stopped states.
	 */
	public void toggleOut() {
		if (state == IntakeState.OUT) {
			stop();
		} else {
			out();
		}
	}
	
	/**
	 * Updates the intake mechanism. Polls color sensor during intake with debounce.
	 */
	@Override
	public void update() {
		// Poll color sensor only when actively intaking
		if (state == IntakeState.IN && colorSensor != null && transfer != null) {
			long now = System.currentTimeMillis();
			
			// Check if enough time has passed since last detection (debounce)
			if (now - lastDetectionTimeMs >= COLOR_DETECTION_DEBOUNCE_MS) {
				MatchSettings.ArtifactColor detected = colorSensor.getArtifactColor();
				
				// Only register if we detect a valid color
				if (detected != MatchSettings.ArtifactColor.UNKNOWN) {
					lastDetectionTimeMs = now;
					lastDetectedColor = detected;
					
					// Register the ball with the transfer, accounting for travel time
					transfer.registerBallFromIntake(detected, now + BALL_TRAVEL_TIME_MS);
				}
			}
		}
	}
	
	/**
	 * Represents the current state of the intake mechanism.
	 */
	public enum IntakeState {
		/** Intake motor running forward to pull artifacts in */
		IN,
		/** Intake motor running in reverse to push artifacts out */
		OUT,
		/** Intake motor stopped */
		STOPPED
	}
}