package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.configuration.Settings;

@TeleOp(name = "Test: Continuous Servo", group = "Tests")
public class CRServoTest extends OpMode {
	CRServo servo;
	double speed = 0.0;

	@Override
	public void init() {
		servo = Settings.Hardware.TRANSFER_WHEEL_SERVO.fromHardwareMap(hardwareMap);
	}
	
	@Override
	public void loop() {
		// Detect dpad up press (rising edge detection to avoid continuous increment)
		if (gamepad1.dpadUpWasPressed()) {
			speed += 0.1;
			servo.setPower(speed);
		}
		if (gamepad1.dpadDownWasPressed()) {
			speed -= 0.1;
			servo.setPower(speed);
		}
		
		telemetry.addData("Speed", "%.3f", speed);
		telemetry.addLine();
		telemetry.addLine("Controls:");
		telemetry.addLine("  Dpad Up: Increase speed");
	}
}
