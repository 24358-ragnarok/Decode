package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.HorizontalLauncher;

@TeleOp(name = "Test: Pitch", group = "Tests")
public class ServoTest extends OpMode {
	Servo pitchServo;
	double pitch = 0.0;
	
	@Override
	public void init() {
		pitchServo = hardwareMap.get(Servo.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
		// Set initial pitch to 0°
		HorizontalLauncher.setPitchDirect(pitchServo, pitch);
	}
	
	@Override
	public void loop() {
		// Detect dpad up press (rising edge detection to avoid continuous increment)
		if (gamepad1.dpadUpWasPressed()) {
			// Increase pitch by 5° on each press
			pitch += 5.0;
			HorizontalLauncher.setPitchDirect(pitchServo, pitch);
		}
		if (gamepad1.dpadDownWasPressed()) {
			pitch -= 5.0;
			HorizontalLauncher.setPitchDirect(pitchServo, pitch);
		}
		
		// Get and display current pitch from servo
		double currentPitch = HorizontalLauncher.getPitchDirect(pitchServo);
		telemetry.addData("Target Pitch", "%.1f°", pitch);
		telemetry.addData("Current Pitch", "%.1f°", currentPitch);
		telemetry.addData("Servo Position", "%.3f", pitchServo.getPosition());
		telemetry.addLine();
		telemetry.addLine("Controls:");
		telemetry.addLine("  Dpad Up: Increase pitch by 5°");
	}
}
