package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;

import java.util.ArrayList;
import java.util.List;

/**
 * Single-mode Velocity PIDF Tuner for flywheel/launcher motors.
 * <p>
 * INIT LOOP: DPAD L/R to select motor.
 * MAIN LOOP:
 * - Bumpers: Cycle P -> I -> D -> F
 * - DPAD U/D: Adjust coefficient by step size
 * - DPAD L/R: Change step size (0.0001 -> 0.1)
 * - X/Y: Set to 3000 / 4500 RPM
 * - A/B: Increment +1000 / +500 RPM
 * - BACK: Zero RPM and reset
 */
@TeleOp(name = "DEEPAK: Motor Velocity PID Tuner", group = "Tests")
public class MotorVelocityPIDTuner extends OpMode {
	
	private static final double DEBOUNCE_TIME = 0.15;
	private static final double[] STEP_MAGNITUDES = {0.0001, 0.001, 0.01, 0.1};
	
	private UnifiedLogging logging;
	private List<MotorInfo> motors;
	private int selectedIndex = 0;
	
	private int selectedCoefficient = 0; // 0=P, 1=I, 2=D, 3=F
	private int stepMagnitudeIndex = 3;
	private double lastCoeffAdjustTime = 0;
	
	private double targetRPM = 0;
	private double currentRPM = 0;
	private PIDFCoefficients pidf;
	
	@Override
	public void init() {
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		motors = discoverMotors();
		
		if (motors.isEmpty()) {
			logging.addLine("⚠️ NO MOTORS FOUND");
		} else {
			logging.addLine("INIT: DPAD ←→ to select motor. START to run.");
		}
		logging.update();
	}
	
	@Override
	public void init_loop() {
		if (motors.isEmpty()) return;
		
		if (gamepad1.dpadLeftWasPressed()) {
			selectedIndex = (selectedIndex - 1 + motors.size()) % motors.size();
		}
		if (gamepad1.dpadRightWasPressed()) {
			selectedIndex = (selectedIndex + 1) % motors.size();
		}
		
		logging.clearDynamic();
		logging.addLine("════ SELECT MOTOR ════");
		for (int i = 0; i < motors.size(); i++) {
			logging.addLine((i == selectedIndex ? "【 " : "  ") + motors.get(i).name + (i == selectedIndex ? " 】" : ""));
		}
		logging.update();
	}
	
	@Override
	public void start() {
		if (motors.isEmpty()) return;
		MotorInfo current = motors.get(selectedIndex);
		
		try {
			pidf = current.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
			if (pidf == null) pidf = new PIDFCoefficients(0, 0, 0, 0);
		} catch (Exception e) {
			pidf = new PIDFCoefficients(0, 0, 0, 0);
		}
		
		targetRPM = 0;
		current.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		current.motor.setPower(0);
	}
	
	@Override
	public void loop() {
		if (motors.isEmpty()) return;
		MotorInfo current = motors.get(selectedIndex);
		double now = getRuntime();
		
		// Menu navigation
		if (gamepad1.rightBumperWasPressed()) selectedCoefficient = (selectedCoefficient + 1) % 4;
		if (gamepad1.leftBumperWasPressed())
			selectedCoefficient = (selectedCoefficient - 1 + 4) % 4;
		if (gamepad1.dpadLeftWasPressed())
			stepMagnitudeIndex = (stepMagnitudeIndex - 1 + STEP_MAGNITUDES.length) % STEP_MAGNITUDES.length;
		if (gamepad1.dpadRightWasPressed())
			stepMagnitudeIndex = (stepMagnitudeIndex + 1) % STEP_MAGNITUDES.length;
		
		// Coefficient Adjustment
		if ((now - lastCoeffAdjustTime) > DEBOUNCE_TIME) {
			double step = STEP_MAGNITUDES[stepMagnitudeIndex];
			if (gamepad1.dpad_up) {
				adjustCoeff(selectedCoefficient, step);
				applyPIDF(current);
				lastCoeffAdjustTime = now;
			}
			if (gamepad1.dpad_down) {
				adjustCoeff(selectedCoefficient, -step);
				applyPIDF(current);
				lastCoeffAdjustTime = now;
			}
		}
		
		// RPM Controls
		if (gamepad1.x) targetRPM = 3000;
		else if (gamepad1.y) targetRPM = 4500;
		if (gamepad1.aWasPressed()) targetRPM += 100;
		if (gamepad1.bWasPressed()) targetRPM -= 100;
		
		// E-Stop
		if (gamepad1.backWasPressed()) {
			targetRPM = 0;
			gamepad1.rumble(200);
		}
		
		targetRPM = Math.max(0, Math.min(10000, targetRPM));
		current.motor.setVelocity(Settings.Launcher.rpmToTicksPerSec(targetRPM));
		currentRPM = Settings.Launcher.ticksPerSecToRPM(current.motor.getVelocity());
		
		displayTelemetry(current);
	}
	
	private void adjustCoeff(int index, double delta) {
		switch (index) {
			case 0:
				pidf.p = Math.max(0, pidf.p + delta);
				break;
			case 1:
				pidf.i = Math.max(0, pidf.i + delta);
				break;
			case 2:
				pidf.d = Math.max(0, pidf.d + delta);
				break;
			case 3:
				pidf.f = Math.max(0, pidf.f + delta);
				break;
		}
	}
	
	private void applyPIDF(MotorInfo current) {
		current.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
	}
	
	private void displayTelemetry(MotorInfo current) {
		logging.clearDynamic();
		
		logging.addLine("════ VELOCITY PIDF TUNER ════");
		logging.addLine("Motor: " + current.name);
		logging.addLine("");
		
		// PIDF and Step Size Display
		String[] coeffNames = {"P", "I", "D", "F"};
		double[] values = {pidf.p, pidf.i, pidf.d, pidf.f};
		
		for (int i = 0; i < 4; i++) {
			logging.addLine(String.format("%s%s: %.4f", (i == selectedCoefficient ? "▶ " : "  "), coeffNames[i], values[i]));
		}
		
		logging.addLine("");
		logging.addLine("Current Step Size: " + STEP_MAGNITUDES[stepMagnitudeIndex]);
		logging.addLine("");
		
		// Live Data
		logging.addData("Target RPM", "%.1f", targetRPM);
		logging.addData("Actual RPM", "%.1f", currentRPM);
		logging.addData("Error", targetRPM - currentRPM);
		logging.addLine("");
		
		// Control Map
		logging.addLine("════ CONTROLS ════");
		logging.addLine("Bumpers: Select P/I/D/F");
		logging.addLine("DPAD ↑↓: Adjust Coefficient");
		logging.addLine("DPAD ←→: Change Step Size");
		logging.addLine("X / Y  : Set 3000 / 4500 RPM");
		logging.addLine("A / B  : ± 100 RPM");
		logging.addLine("BACK   : Set 0 RPM");
		
		logging.update();
	}
	
	private List<MotorInfo> discoverMotors() {
		List<MotorInfo> discovered = new ArrayList<>();
		for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
			MotorInfo info = new MotorInfo();
			info.motor = motor;
			info.name = hardwareMap.getNamesOf(motor).iterator().next();
			discovered.add(info);
		}
		return discovered;
	}
	
	@Override
	public void stop() {
		for (MotorInfo info : motors) {
			info.motor.setPower(0);
			info.motor.setVelocity(0);
		}
	}
	
	private static class MotorInfo {
		DcMotorEx motor;
		String name;
	}
}