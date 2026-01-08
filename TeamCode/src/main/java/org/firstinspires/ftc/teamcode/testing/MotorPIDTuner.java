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
 * Interactive Motor PID Tuning OpMode for precise PID coefficient adjustment.
 * <p>
 * Supports two test modes:
 * <ul>
 * <li><b>VELOCITY MODE</b>: Tests PIDF coefficients using velocity control (for
 * flywheels/launchers)</li>
 * <li><b>POSITION MODE</b>: Tests PIDF coefficients using position control (for
 * transfer mechanisms)</li>
 * </ul>
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li><b>INIT_LOOP:</b></li>
 * <li>DPAD LEFT/RIGHT: Select motor</li>
 * <li><b>MAIN LOOP:</b></li>
 * <li>START: Toggle between VELOCITY and POSITION test modes</li>
 * <li>BUMPERS: Cycle through P, I, D, F coefficients</li>
 * <li>DPAD UP/DOWN: Adjust selected coefficient by current step size</li>
 * <li>DPAD LEFT/RIGHT: Change step size magnitude (0.1 → 1.0 → 10.0 →
 * 0.1...)</li>
 * <li><b>--- VELOCITY MODE ---</b></li>
 * <li>X (hold): Run at 3000 RPM</li>
 * <li>Y (hold): Run at 4500 RPM</li>
 * <li>A: Increment target by 1000 RPM</li>
 * <li>B: Increment target by 500 RPM</li>
 * <li><b>--- POSITION MODE ---</b></li>
 * <li>X (hold): Move +1000 ticks</li>
 * <li>Y (hold): Move +500 ticks</li>
 * <li>A: Increment target by 1000 ticks</li>
 * <li>B: Increment target by 500 ticks</li>
 * <li><b>--- COMMON ---</b></li>
 * <li>BACK: Stop motor and reset to zero</li>
 * </ul>
 * <p>
 * <b>FEATURES:</b>
 * <ul>
 * <li>Auto-discovers all connected motors</li>
 * <li>Loads current PIDF coefficients on start</li>
 * <li>Real-time coefficient editing with visual feedback</li>
 * <li>Separate PIDF sets for velocity and position modes</li>
 * <li>Current velocity/position and target display</li>
 * <li>Step size magnitude adjustment for fine/coarse tuning</li>
 * </ul>
 */
@TeleOp(name = "Debug: Motor PID Tuner", group = "Tests")
public class MotorPIDTuner extends OpMode {
	
	private static final double DEBOUNCE_TIME = 0.15; // seconds
	private static final double[] STEP_MAGNITUDES = {0.0001, 0.001, 0.01, 0.1};
	
	UnifiedLogging logging;
	private List<MotorInfo> motors;
	private int selectedIndex = 0;
	
	// PID editing state
	private int selectedCoefficient = 0; // 0=P, 1=I, 2=D, 3=F
	private int stepMagnitudeIndex = 3; // Index into STEP_MAGNITUDES
	private double lastCoeffAdjustTime = 0;
	private double lastPositionIncrementTime = 0;
	
	// Test mode
	private TestMode testMode = TestMode.VELOCITY;
	
	// Velocity mode state
	private double targetRPM = 0;
	private double currentRPM = 0;
	
	// Position mode state
	private int targetPosition = 0;
	private int currentPosition = 0;
	
	// PIDF coefficients (separate for each mode)
	private PIDFCoefficients velocityPIDF;
	private PIDFCoefficients positionPIDF;
	
	@Override
	public void init() {
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		motors = discoverMotors();
		
		if (motors.isEmpty()) {
			logging.addLine("⚠️ NO MOTORS FOUND!");
			logging.addLine("Check your hardware configuration.");
		} else {
			logging.addLine("✅ Found " + motors.size() + " motor(s)");
			logging.addLine("");
			for (MotorInfo info : motors) {
				logging.addLine("  • " + info.name);
			}
			
			logging.addLine("");
			logging.addLine("INIT: Use DPAD ←→ to select motor");
			logging.addLine("Press START when ready to begin tuning");
		}
		
		logging.update();
	}
	
	@Override
	public void init_loop() {
		if (motors.isEmpty()) {
			return;
		}
		
		// Motor selection during init
		if (gamepad1.dpadLeftWasPressed()) {
			selectedIndex = (selectedIndex - 1 + motors.size()) % motors.size();
		}
		if (gamepad1.dpadRightWasPressed()) {
			selectedIndex = (selectedIndex + 1) % motors.size();
		}
		
		// Display selection
		logging.clearDynamic();
		logging.addLine("═══════ SELECT MOTOR ═══════");
		logging.addLine("");
		for (int i = 0; i < motors.size(); i++) {
			if (i == selectedIndex) {
				logging.addLine("【" + motors.get(i).name + "】");
			} else {
				logging.addLine("  " + motors.get(i).name);
			}
		}
		logging.addLine("");
		logging.addLine("DPAD ←→: Select");
		logging.addLine("START: Begin tuning");
		logging.update();
	}
	
	@Override
	public void start() {
		if (motors.isEmpty()) {
			return;
		}
		
		MotorInfo current = motors.get(selectedIndex);
		
		// Load current PIDF coefficients for both modes
		try {
			velocityPIDF = current.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
			if (velocityPIDF == null) {
				velocityPIDF = new PIDFCoefficients(0, 0, 0, 0);
			}
		} catch (Exception e) {
			velocityPIDF = new PIDFCoefficients(0, 0, 0, 0);
		}
		
		try {
			positionPIDF = current.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
			if (positionPIDF == null) {
				positionPIDF = new PIDFCoefficients(0, 0, 0, 0);
			}
		} catch (Exception e) {
			positionPIDF = new PIDFCoefficients(0, 0, 0, 0);
		}
		
		// Initialize targets
		targetRPM = 0;
		targetPosition = current.motor.getCurrentPosition();
		
		// Apply initial mode
		applyTestMode(current);
	}
	
	@Override
	public void loop() {
		if (motors.isEmpty()) {
			logging.addLine("⚠️ NO MOTORS FOUND!");
			logging.update();
			return;
		}
		
		MotorInfo current = motors.get(selectedIndex);
		double now = getRuntime();
		
		// === MODE TOGGLE ===
		if (gamepad1.startWasPressed()) {
			testMode = (testMode == TestMode.VELOCITY) ? TestMode.POSITION : TestMode.VELOCITY;
			applyTestMode(current);
		}
		
		// === COEFFICIENT SELECTION ===
		if (gamepad1.rightBumperWasPressed()) {
			selectedCoefficient = (selectedCoefficient + 1) % 4;
		}
		if (gamepad1.leftBumperWasPressed()) {
			selectedCoefficient = (selectedCoefficient - 1 + 4) % 4;
		}
		
		// === STEP SIZE MAGNITUDE ===
		if (gamepad1.dpadLeftWasPressed()) {
			stepMagnitudeIndex = (stepMagnitudeIndex - 1 + STEP_MAGNITUDES.length) % STEP_MAGNITUDES.length;
		}
		if (gamepad1.dpadRightWasPressed()) {
			stepMagnitudeIndex = (stepMagnitudeIndex + 1) % STEP_MAGNITUDES.length;
		}
		
		// === COEFFICIENT ADJUSTMENT ===
		if ((now - lastCoeffAdjustTime) > DEBOUNCE_TIME) {
			double stepSize = STEP_MAGNITUDES[stepMagnitudeIndex];
			PIDFCoefficients activePIDF = (testMode == TestMode.VELOCITY) ? velocityPIDF : positionPIDF;
			
			if (gamepad1.dpad_up) {
				adjustCoefficient(activePIDF, selectedCoefficient, stepSize);
				applyPIDF(current);
				lastCoeffAdjustTime = now;
			}
			if (gamepad1.dpad_down) {
				adjustCoefficient(activePIDF, selectedCoefficient, -stepSize);
				applyPIDF(current);
				lastCoeffAdjustTime = now;
			}
		}
		
		// === TEST CONTROLS ===
		if (testMode == TestMode.VELOCITY) {
			handleVelocityModeInput(current);
		} else {
			handlePositionModeInput(current);
		}
		
		// === EMERGENCY STOP ===
		if (gamepad1.backWasPressed()) {
			current.motor.setPower(0);
			current.motor.setVelocity(0);
			targetRPM = 0;
			targetPosition = current.motor.getCurrentPosition();
			gamepad1.rumble(200);
		}
		
		// === UPDATE READINGS ===
		if (testMode == TestMode.VELOCITY) {
			currentRPM = Settings.Launcher.ticksPerSecToRPM(current.motor.getVelocity());
		} else {
			currentPosition = current.motor.getCurrentPosition();
		}
		
		// === DISPLAY ===
		displayTelemetry(current);
	}
	
	private void handleVelocityModeInput(MotorInfo current) {
		// Ensure motor is in velocity mode
		if (current.motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
			current.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
		
		// Hold buttons for continuous RPM (set directly, don't increment)
		if (gamepad1.x) {
			targetRPM = 3000;
		} else if (gamepad1.y) {
			targetRPM = 4500;
		}
		
		// Increment buttons (only on press, not hold)
		if (gamepad1.aWasPressed()) {
			targetRPM += 1000;
		}
		if (gamepad1.bWasPressed()) {
			targetRPM += 500;
		}
		
		// Clamp to reasonable range
		targetRPM = Math.max(0, Math.min(10000, targetRPM));
		
		// Apply velocity
		double targetTPS = Settings.Launcher.rpmToTicksPerSec(targetRPM);
		current.motor.setVelocity(targetTPS);
	}
	
	private void handlePositionModeInput(MotorInfo current) {
		// Ensure motor is in position mode
		if (current.motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
			current.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			current.motor.setPower(0.5); // Reset power for position mode
		}
		
		// Hold buttons for continuous movement (increment on each loop iteration)
		// Use debouncing to prevent too-fast increments
		double now = getRuntime();
		if (gamepad1.x && (now - lastPositionIncrementTime) > 0.1) {
			targetPosition += 1000;
			lastPositionIncrementTime = now;
		} else if (gamepad1.y && (now - lastPositionIncrementTime) > 0.1) {
			targetPosition += 500;
			lastPositionIncrementTime = now;
		}
		
		// Increment buttons (only on press, not hold)
		if (gamepad1.aWasPressed()) {
			targetPosition += 100;
		}
		if (gamepad1.bWasPressed()) {
			targetPosition += 10;
		}
		
		// Apply position
		current.motor.setTargetPosition(targetPosition);
	}
	
	private void adjustCoefficient(PIDFCoefficients pidf, int index, double delta) {
		switch (index) {
			case 0: // P
				pidf.p = Math.max(0, pidf.p + delta);
				break;
			case 1: // I
				pidf.i = Math.max(0, pidf.i + delta);
				break;
			case 2: // D
				pidf.d = Math.max(0, pidf.d + delta);
				break;
			case 3: // F
				pidf.f = Math.max(0, pidf.f + delta);
				break;
		}
	}
	
	private void applyPIDF(MotorInfo current) {
		try {
			if (testMode == TestMode.VELOCITY) {
				current.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
			} else {
				current.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, positionPIDF);
			}
		} catch (Exception e) {
			// PIDF not supported on this motor
		}
	}
	
	private void applyTestMode(MotorInfo current) {
		// Stop motor first
		current.motor.setPower(0);
		current.motor.setVelocity(0);
		
		if (testMode == TestMode.VELOCITY) {
			current.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			targetRPM = 0;
			applyPIDF(current);
		} else {
			current.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			targetPosition = current.motor.getCurrentPosition();
			current.motor.setPower(0.5);
			applyPIDF(current);
		}
	}
	
	private void displayTelemetry(MotorInfo current) {
		logging.clearDynamic();
		
		// Header
		logging.addLine("═══════ MOTOR PID TUNER ═══════");
		logging.addLine("");
		
		// Motor name
		logging.addLine("Motor: " + current.name);
		logging.addLine("");
		
		// Mode indicator
		String modeStr = (testMode == TestMode.VELOCITY) ? "⚡ VELOCITY MODE" : "📍 POSITION MODE";
		logging.addLine(modeStr);
		logging.addLine("");
		
		// PIDF coefficients
		PIDFCoefficients activePIDF = (testMode == TestMode.VELOCITY) ? velocityPIDF : positionPIDF;
		String[] coeffNames = {"P", "I", "D", "F"};
		
		logging.addLine("═══════ PIDF COEFFICIENTS ═══════");
		for (int i = 0; i < 4; i++) {
			String prefix = (i == selectedCoefficient) ? "▶ " : "  ";
			double value = 0;
			switch (i) {
				case 0:
					value = activePIDF.p;
					break;
				case 1:
					value = activePIDF.i;
					break;
				case 2:
					value = activePIDF.d;
					break;
				case 3:
					value = activePIDF.f;
					break;
			}
			logging.addLine(String.format("%s%s: %.4f", prefix, coeffNames[i], value));
		}
		logging.addLine("");
		logging.addLine("Step size: " + STEP_MAGNITUDES[stepMagnitudeIndex]);
		logging.addLine("");
		
		// Test values
		if (testMode == TestMode.VELOCITY) {
			logging.addLine("═══════ VELOCITY TEST ═══════");
			logging.addData("Target RPM", "%.1f", targetRPM);
			logging.addData("Current RPM", "%.1f", currentRPM);
			logging.addData("error", Math.abs(targetRPM - currentRPM));
			logging.addData("Target TPS", "%.1f", Settings.Launcher.rpmToTicksPerSec(targetRPM));
			logging.addData("Current TPS", "%.1f", current.motor.getVelocity());
		} else {
			logging.addLine("═══════ POSITION TEST ═══════");
			logging.addData("Target", targetPosition);
			logging.addData("Current", currentPosition);
			logging.addData("error", targetPosition - currentPosition);
			logging.addData("Busy", current.motor.isBusy() ? "YES" : "no");
		}
		logging.addLine("");
		
		// Controls reminder
		logging.addLine("═══════ CONTROLS ═══════");
		logging.addLine("START: Toggle mode");
		logging.addLine("Bumpers: Select P/I/D/F");
		logging.addLine("↑↓: Adjust coefficient");
		logging.addLine("←→: Change step size");
		if (testMode == TestMode.VELOCITY) {
			logging.addLine("X=3000 RPM | Y=4500 RPM");
			logging.addLine("A=+1000 | B=+500");
		} else {
			logging.addLine("X=+1000 ticks | Y=+500 ticks");
			logging.addLine("A=+100 | B=+10");
		}
		logging.addLine("BACK: Stop & reset");
		
		logging.update();
	}
	
	/**
	 * Discovers all motors in the hardware map.
	 */
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
		// Stop all motors on exit
		for (MotorInfo info : motors) {
			info.motor.setPower(0);
			info.motor.setVelocity(0);
		}
	}
	
	private enum TestMode {
		VELOCITY, // Velocity control (RUN_USING_ENCODER)
		POSITION // Position control (RUN_TO_POSITION)
	}
	
	/**
	 * Container for motor information.
	 */
	private static class MotorInfo {
		DcMotorEx motor;
		String name;
	}
}
