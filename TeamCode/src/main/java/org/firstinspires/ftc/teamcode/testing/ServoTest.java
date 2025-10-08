package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.Settings;

@TeleOp(name = "Test: Servo", group = "Tests")
public class ServoTest extends OpMode {
	Servo servo;
	double pos = 0.0;
	
	@Override
	public void init() {
		servo = hardwareMap.get(Servo.class, Settings.HardwareIDs.LAUNCHER_PITCH_SERVO);
	}
	
	@Override
	public void loop() {
		if (gamepad1.left_trigger > 0) {
			pos = -gamepad1.left_stick_y;
		}
		servo.setPosition(pos);
		telemetry.addData("Position", pos);
	}
}
