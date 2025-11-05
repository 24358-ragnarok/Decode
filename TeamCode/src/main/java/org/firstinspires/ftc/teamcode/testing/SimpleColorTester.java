package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.ColorSensor;

@TeleOp(name = "Test: Color", group = "Tests")
public class SimpleColorTester extends OpMode {
	RevColorSensorV3 colorSensorV3;
	ColorSensor wrapper;
	
	@Override
	public void init() {
		colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, Settings.HardwareIDs.TRANSFER_COLOR_SENSOR);
		wrapper = new ColorSensor(colorSensorV3);
	}
	
	@Override
	public void loop() {
		double[] sensed = new double[3];
		sensed[0] = colorSensorV3.red();
		sensed[1] = colorSensorV3.green();
		sensed[2] = colorSensorV3.blue();
		telemetry.addData("R", colorSensorV3.red());
		telemetry.addData("G", colorSensorV3.green());
		telemetry.addData("B", colorSensorV3.blue());
		telemetry.addData("detected", wrapper.getArtifactColor());
		telemetry.addData("distance to green", wrapper.computeDistance(sensed, Settings.ColorSensor.GREEN_TARGET));
		telemetry.addData("distance to purple", wrapper.computeDistance(sensed, Settings.ColorSensor.PURPLE_TARGET));
		telemetry.addData("physical distance inches", colorSensorV3.getDistance(DistanceUnit.INCH));
	}
}
