package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;

import java.util.ArrayList;
import java.util.List;

/**
 * Interactive Motor Debug OpMode for testing and tuning motors.
 * <p>
 * Supports two control modes:
 * <ul>
 * <li><b>POWER (Continuous)</b>: Direct power control like flywheel/intake</li>
 * <li><b>POSITION</b>: Encoder-based positioning like transfer mechanisms</li>
 * </ul>
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li>DPAD LEFT/RIGHT: Select previous/next motor</li>
 * <li>START: Toggle between POWER and POSITION mode</li>
 * <li><b>--- POWER MODE ---</b></li>
 * <li>RIGHT STICK Y: Direct power control (-1 to 1)</li>
 * <li>DPAD UP/DOWN: Adjust power setpoint ±0.05</li>
 * <li>BUMPERS: Adjust power setpoint ±0.01</li>
 * <li>A: Set power to 0</li>
 * <li>B: Set power to 0.5</li>
 * <li>Y: Set power to 1.0</li>
 * <li><b>--- POSITION MODE ---</b></li>
 * <li>DPAD UP/DOWN: Move target ±100 ticks</li>
 * <li>BUMPERS: Move target ±10 ticks</li>
 * <li>TRIGGERS: Move target ±1 tick</li>
 * <li>A: Go to position 0</li>
 * <li>B: Set current position as zero</li>
 * <li>Y: Hold current position</li>
 * <li><b>--- COMMON ---</b></li>
 * <li>X: Toggle motor direction</li>
 * <li>LEFT STICK BUTTON: Cycle zero power behavior</li>
 * <li>RIGHT STICK BUTTON: Cycle run mode (position only)</li>
 * <li>BACK: Emergency stop (coast)</li>
 * </ul>
 * <p>
 * <b>FEATURES:</b>
 * <ul>
 * <li>Auto-discovers all connected motors</li>
 * <li>Real-time encoder position and velocity</li>
 * <li>Current draw monitoring (if supported)</li>
 * <li>Direction, run mode, zero power behavior display</li>
 * <li>Position error and busy state for RUN_TO_POSITION</li>
 * </ul>
 */
@TeleOp(name = "Debug: Motor", group = "Tests")
public class MotorDebug extends OpMode {
	
	private static final double COARSE_DEBOUNCE = 0.12; // seconds
	private static final double FINE_DEBOUNCE = 0.04; // seconds
	UnifiedLogging logging;
	private List<MotorInfo> motors;
	private int selectedIndex = 0;
	// Control state
	private ControlMode controlMode = ControlMode.POWER;
	private double commandedPower = 0.0;
	private int targetPosition = 0;
	private double positionPower = 0.5; // Power used in RUN_TO_POSITION mode
	
	// Timing
	private double lastCoarseAdjust = 0;
	private double lastFineAdjust = 0;
	
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
			
			// Initialize to first motor's current state
			MotorInfo first = motors.get(0);
			targetPosition = first.motor.getCurrentPosition();
		}
		
		logging.addLine("");
		logging.addLine("Controls:");
		logging.addLine("  DPAD ←→: Select motor");
		logging.addLine("  START: Toggle POWER/POSITION mode");
		logging.addLine("  X: Toggle direction");
		logging.addLine("  BACK: Emergency stop");
		logging.update();
	}
	
	@Override
	public void init_loop() {
		// Keep motors disabled during init
		for (MotorInfo info : motors) {
			info.motor.setPower(0);
		}
	}
	
	@Override
	public void start() {
		// Set initial state when starting
		if (!motors.isEmpty()) {
			MotorInfo current = motors.get(selectedIndex);
			targetPosition = current.motor.getCurrentPosition();
			applyControlMode(current);
		}
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
		
		// === MOTOR SELECTION ===
		if (gamepad1.dpadLeftWasPressed()) {
			// Stop current motor before switching
			current.motor.setPower(0);
			
			selectedIndex = (selectedIndex - 1 + motors.size()) % motors.size();
			current = motors.get(selectedIndex);
			targetPosition = current.motor.getCurrentPosition();
			commandedPower = 0;
			applyControlMode(current);
		}
		if (gamepad1.dpadRightWasPressed()) {
			// Stop current motor before switching
			current.motor.setPower(0);
			
			selectedIndex = (selectedIndex + 1) % motors.size();
			current = motors.get(selectedIndex);
			targetPosition = current.motor.getCurrentPosition();
			commandedPower = 0;
			applyControlMode(current);
		}
		
		// === MODE TOGGLE ===
		if (gamepad1.startWasPressed()) {
			controlMode = (controlMode == ControlMode.POWER) ? ControlMode.POSITION : ControlMode.POWER;
			commandedPower = 0;
			targetPosition = current.motor.getCurrentPosition();
			applyControlMode(current);
		}
		
		// === EMERGENCY STOP ===
		if (gamepad1.backWasPressed()) {
			current.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			current.motor.setPower(0);
			commandedPower = 0;
			gamepad1.rumble(200);
		}
		
		// === MODE-SPECIFIC CONTROLS ===
		if (controlMode == ControlMode.POWER) {
			handlePowerModeInput(current, now);
		} else {
			handlePositionModeInput(current, now);
		}
		
		// === COMMON CONTROLS ===
		// Direction toggle
		if (gamepad1.xWasPressed()) {
			current.motor.setDirection(
					current.motor.getDirection() == DcMotorSimple.Direction.FORWARD
							? DcMotorSimple.Direction.REVERSE
							: DcMotorSimple.Direction.FORWARD);
		}
		
		// Zero power behavior cycle
		if (gamepad1.leftStickButtonWasPressed()) {
			DcMotor.ZeroPowerBehavior currentBehavior = current.motor.getZeroPowerBehavior();
			current.motor.setZeroPowerBehavior(
					currentBehavior == DcMotor.ZeroPowerBehavior.BRAKE
							? DcMotor.ZeroPowerBehavior.FLOAT
							: DcMotor.ZeroPowerBehavior.BRAKE);
		}
		
		// === logging ===
		displayTelemetry(current);
	}
	
	private void handlePowerModeInput(MotorInfo current, double now) {
		// Direct stick control (overrides setpoint when stick is moved)
		if (Math.abs(gamepad1.right_stick_y) > 0.1) {
			commandedPower = -gamepad1.right_stick_y; // Inverted for intuitive control
		} else {
			// Coarse adjustment (±0.05)
			if (gamepad1.dpad_up && (now - lastCoarseAdjust) > COARSE_DEBOUNCE) {
				commandedPower += 0.05;
				lastCoarseAdjust = now;
			}
			if (gamepad1.dpad_down && (now - lastCoarseAdjust) > COARSE_DEBOUNCE) {
				commandedPower -= 0.05;
				lastCoarseAdjust = now;
			}
			
			// Fine adjustment (±0.01)
			if (gamepad1.right_bumper && (now - lastFineAdjust) > FINE_DEBOUNCE) {
				commandedPower += 0.01;
				lastFineAdjust = now;
			}
			if (gamepad1.left_bumper && (now - lastFineAdjust) > FINE_DEBOUNCE) {
				commandedPower -= 0.01;
				lastFineAdjust = now;
			}
		}
		
		// Preset powers
		if (gamepad1.aWasPressed()) {
			commandedPower = 0.0;
		}
		if (gamepad1.bWasPressed()) {
			commandedPower = 0.5;
		}
		if (gamepad1.yWasPressed()) {
			commandedPower = 1.0;
		}
		
		// Clamp and apply
		commandedPower = Math.max(-1.0, Math.min(1.0, commandedPower));
		current.motor.setPower(commandedPower);
	}
	
	private void handlePositionModeInput(MotorInfo current, double now) {
		// Coarse adjustment (±100 ticks)
		if (gamepad1.dpad_up && (now - lastCoarseAdjust) > COARSE_DEBOUNCE) {
			targetPosition += 100;
			lastCoarseAdjust = now;
		}
		if (gamepad1.dpad_down && (now - lastCoarseAdjust) > COARSE_DEBOUNCE) {
			targetPosition -= 100;
			lastCoarseAdjust = now;
		}
		
		// Medium adjustment (±10 ticks)
		if (gamepad1.right_bumper && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			targetPosition += 10;
			lastFineAdjust = now;
		}
		if (gamepad1.left_bumper && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			targetPosition -= 10;
			lastFineAdjust = now;
		}
		
		// Fine adjustment (±1 tick) via triggers
		if (gamepad1.right_trigger > 0.5 && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			targetPosition += 1;
			lastFineAdjust = now;
		}
		if (gamepad1.left_trigger > 0.5 && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			targetPosition -= 1;
			lastFineAdjust = now;
		}
		
		// Position presets
		if (gamepad1.aWasPressed()) {
			targetPosition = 0;
		}
		if (gamepad1.bWasPressed()) {
			// Reset encoder to zero at current position
			current.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			current.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			targetPosition = 0;
		}
		if (gamepad1.yWasPressed()) {
			// Hold current position
			targetPosition = current.motor.getCurrentPosition();
		}
		
		// Adjust position power with right stick
		if (Math.abs(gamepad1.right_stick_y) > 0.1) {
			positionPower = Math.abs(-gamepad1.right_stick_y);
		}
		
		// Run mode cycle (for position mode)
		if (gamepad1.rightStickButtonWasPressed()) {
			DcMotor.RunMode currentMode = current.motor.getMode();
			if (currentMode == DcMotor.RunMode.RUN_TO_POSITION) {
				current.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			} else if (currentMode == DcMotor.RunMode.RUN_USING_ENCODER) {
				current.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			} else {
				current.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}
		}
		
		// Apply position
		current.motor.setTargetPosition(targetPosition);
		current.motor.setPower(positionPower);
	}
	
	private void applyControlMode(MotorInfo current) {
		if (controlMode == ControlMode.POWER) {
			current.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			current.motor.setPower(0);
		} else {
			current.motor.setTargetPosition(current.motor.getCurrentPosition());
			current.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			current.motor.setPower(positionPower);
		}
	}
	
	private void displayTelemetry(MotorInfo current) {
		logging.clearDynamic();
		
		// Header
		logging.addLine("═══════ MOTOR DEBUG ═══════");
		logging.addLine("");
		
		// Motor selector
		StringBuilder selector = new StringBuilder();
		for (int i = 0; i < motors.size(); i++) {
			if (i == selectedIndex) {
				selector.append("【").append(motors.get(i).name).append("】 ");
			} else {
				selector.append(motors.get(i).name).append(" ");
			}
		}
		logging.addLine(selector.toString());
		logging.addLine("");
		
		// Mode indicator
		String modeStr = (controlMode == ControlMode.POWER) ? "⚡ POWER MODE" : "📍 POSITION MODE";
		logging.addLine(modeStr);
		logging.addLine("");
		
		// Mode-specific info
		if (controlMode == ControlMode.POWER) {
			displayPowerModeTelemetry(current);
		} else {
			displayPositionModeTelemetry(current);
		}
		
		// Common info
		logging.addLine("═══════ MOTOR STATE ═══════");
		logging.addData("Direction", current.motor.getDirection());
		logging.addData("Zero Power", current.motor.getZeroPowerBehavior());
		logging.addData("Run Mode", current.motor.getMode());
		logging.addData("Busy", current.motor.isBusy() ? "YES" : "no");
		
		// Extended info
		if (current.motor instanceof DcMotorEx) {
			DcMotorEx motorEx = current.motor;
			logging.addData("Velocity", "%.1f ticks/s", motorEx.getVelocity());
			try {
				logging.addData("Current", "%.2f A",
						motorEx.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS));
			} catch (Exception e) {
				// Current monitoring not supported
			}
		}
		logging.addLine("");
		
		// Controls reminder
		logging.addLine("═══════ CONTROLS ═══════");
		if (controlMode == ControlMode.POWER) {
			logging.addLine("R-Stick: Power | ↑↓ ±0.05 | Bump ±0.01");
			logging.addLine("A=0 | B=0.5 | Y=1.0");
		} else {
			logging.addLine("↑↓ ±100 | Bump ±10 | Trig ±1");
			logging.addLine("A=goto 0 | B=reset enc | Y=hold");
			logging.addLine("R-Stick: Adj power | R3=cycle mode");
		}
		logging.addLine("START=mode | X=dir | L3=brake | BACK=stop");
		
		logging.update();
	}
	
	private void displayPowerModeTelemetry(MotorInfo current) {
		logging.addLine("═══════ POWER ═══════");
		logging.addData("Commanded", "%.3f", commandedPower);
		
		// Visual power bar
		int barLength = 20;
		int center = barLength / 2;
		int filledLength = (int) (Math.abs(commandedPower) * center);
		StringBuilder bar = new StringBuilder("[");
		for (int i = 0; i < barLength; i++) {
			if (i == center) {
				bar.append("|");
			} else if (commandedPower > 0 && i > center && i <= center + filledLength) {
				bar.append("▶");
			} else if (commandedPower < 0 && i < center && i >= center - filledLength) {
				bar.append("◀");
			} else {
				bar.append("─");
			}
		}
		bar.append("]");
		logging.addLine(bar.toString());
		
		logging.addData("Encoder", current.motor.getCurrentPosition());
		logging.addLine("");
	}
	
	private void displayPositionModeTelemetry(MotorInfo current) {
		logging.addLine("═══════ POSITION ═══════");
		int currentPos = current.motor.getCurrentPosition();
		int error = targetPosition - currentPos;
		
		logging.addData("Target", targetPosition);
		logging.addData("Current", currentPos);
		logging.addData("Error", "%d ticks", error);
		logging.addData("Power", "%.2f", positionPower);
		logging.addData("Deadzone", current.motor.getTargetPositionTolerance());
		
		// Visual position indicator
		int range = 500; // Display range
		int barLength = 20;
		int normalizedPos = Math.max(-range, Math.min(range, currentPos));
		int normalizedTarget = Math.max(-range, Math.min(range, targetPosition));
		int posIndex = (int) ((normalizedPos + range) / (2.0 * range) * (barLength - 1));
		int targetIndex = (int) ((normalizedTarget + range) / (2.0 * range) * (barLength - 1));
		
		StringBuilder bar = new StringBuilder("[");
		for (int i = 0; i < barLength; i++) {
			if (i == targetIndex && i == posIndex) {
				bar.append("◉"); // Both at same position
			} else if (i == targetIndex) {
				bar.append("○"); // Target
			} else if (i == posIndex) {
				bar.append("●"); // Current
			} else if (i == barLength / 2) {
				bar.append("|"); // Zero marker
			} else {
				bar.append("─");
			}
		}
		bar.append("]");
		logging.addLine(bar.toString());
		logging.addLine("●=current ○=target |=zero");
		logging.addLine("");
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
		}
	}
	
	private enum ControlMode {
		POWER, // Continuous power control (flywheel style)
		POSITION // Encoder position control (transfer style)
	}
	
	/**
	 * Container for motor information.
	 */
	private static class MotorInfo {
		DcMotorEx motor;
		String name;
	}
}
