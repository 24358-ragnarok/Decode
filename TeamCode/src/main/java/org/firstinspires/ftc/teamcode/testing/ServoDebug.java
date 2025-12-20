package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.UnifiedLogging;

import java.util.ArrayList;
import java.util.List;

/**
 * Interactive Servo Debug OpMode for testing and tuning servos.
 * <p>
 * Allows you to select any servo connected to the robot, view detailed
 * information about it, and control its position with fine precision.
 * <p>
 * <b>CONTROLS:</b>
 * <ul>
 * <li>DPAD LEFT/RIGHT: Select previous/next servo</li>
 * <li>DPAD UP/DOWN: Adjust position (coarse: ±0.05)</li>
 * <li>LEFT/RIGHT BUMPER: Adjust position (fine: ±0.01)</li>
 * <li>LEFT/RIGHT TRIGGER: Adjust position (ultra-fine: ±0.001)</li>
 * <li>A: Go to position 0.0</li>
 * <li>B: Go to position 0.5</li>
 * <li>Y: Go to position 1.0</li>
 * <li>X: Toggle servo enable/disable (PWM)</li>
 * <li>LEFT STICK BUTTON: Reverse servo direction</li>
 * <li>RIGHT STICK BUTTON: Reset to default direction</li>
 * <li>BACK: Sweep test (0→1→0)</li>
 * </ul>
 * <p>
 * <b>FEATURES:</b>
 * <ul>
 * <li>Auto-discovers all connected servos</li>
 * <li>Shows position, PWM range, direction, controller info</li>
 * <li>Fine-grained position control with multiple precision levels</li>
 * <li>Sweep test for range verification</li>
 * <li>PWM enable/disable for testing without movement</li>
 * </ul>
 */
@TeleOp(name = "Debug: Servo", group = "Tests")
public class ServoDebug extends OpMode {
	
	private static final double COARSE_DEBOUNCE = 0.15; // seconds
	private static final double FINE_DEBOUNCE = 0.05;   // seconds
	private final double sweepSpeed = 0.005;
	UnifiedLogging logging;
	private List<ServoInfo> servos;
	private int selectedIndex = 0;
	private double commandedPosition = 0.5;
	private boolean isPwmEnabled = true;
	private boolean isSweeping = false;
	private double sweepDirection = 1.0;
	// Timing for button debouncing
	private double lastCoarseAdjust = 0;
	private double lastFineAdjust = 0;
	
	@Override
	public void init() {
		logging = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
		servos = discoverServos();
		
		if (servos.isEmpty()) {
			logging.addLine("⚠️ NO SERVOS FOUND!");
			logging.addLine("Check your hardware configuration.");
		} else {
			logging.addLine("✅ Found " + servos.size() + " servo(s)");
			logging.addLine("");
			for (ServoInfo info : servos) {
				logging.addLine("  • " + info.name);
			}
			
			// Initialize to first servo's current position
			commandedPosition = servos.get(0).servo.getPosition();
		}
		
		logging.addLine("");
		logging.addLine("Controls:");
		logging.addLine("  DPAD ←→: Select servo");
		logging.addLine("  DPAD ↑↓: Position ±0.05");
		logging.addLine("  BUMPERS: Position ±0.01");
		logging.addLine("  TRIGGERS: Position ±0.001");
		logging.addLine("  A/B/Y: Goto 0.0/0.5/1.0");
		logging.addLine("  X: Toggle PWM | BACK: Sweep");
		logging.update();
	}
	
	@Override
	public void loop() {
		if (servos.isEmpty()) {
			logging.addLine("⚠️ NO SERVOS FOUND!");
			logging.update();
			return;
		}
		
		ServoInfo current = servos.get(selectedIndex);
		double now = getRuntime();
		
		// === SERVO SELECTION ===
		if (gamepad1.dpadLeftWasPressed()) {
			selectedIndex = (selectedIndex - 1 + servos.size()) % servos.size();
			commandedPosition = servos.get(selectedIndex).servo.getPosition();
			isSweeping = false;
		}
		if (gamepad1.dpadRightWasPressed()) {
			selectedIndex = (selectedIndex + 1) % servos.size();
			commandedPosition = servos.get(selectedIndex).servo.getPosition();
			isSweeping = false;
		}
		
		// === POSITION ADJUSTMENT ===
		// Coarse adjustment (±0.05)
		if (gamepad1.dpad_up && (now - lastCoarseAdjust) > COARSE_DEBOUNCE) {
			commandedPosition += 0.05;
			lastCoarseAdjust = now;
			isSweeping = false;
		}
		if (gamepad1.dpad_down && (now - lastCoarseAdjust) > COARSE_DEBOUNCE) {
			commandedPosition -= 0.05;
			lastCoarseAdjust = now;
			isSweeping = false;
		}
		
		// Fine adjustment (±0.01)
		if (gamepad1.right_bumper && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			commandedPosition += 0.01;
			lastFineAdjust = now;
			isSweeping = false;
		}
		if (gamepad1.left_bumper && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			commandedPosition -= 0.01;
			lastFineAdjust = now;
			isSweeping = false;
		}
		
		// Ultra-fine adjustment (±0.001) via triggers
		if (gamepad1.right_trigger > 0.5 && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			commandedPosition += 0.001;
			lastFineAdjust = now;
			isSweeping = false;
		}
		if (gamepad1.left_trigger > 0.5 && (now - lastFineAdjust) > FINE_DEBOUNCE) {
			commandedPosition -= 0.001;
			lastFineAdjust = now;
			isSweeping = false;
		}
		
		// === PRESET POSITIONS ===
		if (gamepad1.aWasPressed()) {
			commandedPosition = 0.0;
			isSweeping = false;
		}
		if (gamepad1.bWasPressed()) {
			commandedPosition = 0.5;
			isSweeping = false;
		}
		if (gamepad1.yWasPressed()) {
			commandedPosition = 1.0;
			isSweeping = false;
		}
		
		// === PWM TOGGLE ===
		if (gamepad1.xWasPressed()) {
			isPwmEnabled = !isPwmEnabled;
			if (current.servo instanceof ServoImplEx) {
				ServoImplEx servoEx = (ServoImplEx) current.servo;
				if (isPwmEnabled) {
					servoEx.setPwmEnable();
				} else {
					servoEx.setPwmDisable();
				}
			}
		}
		
		// === DIRECTION CONTROL ===
		if (gamepad1.leftStickButtonWasPressed()) {
			current.servo.setDirection(
					current.servo.getDirection() == Servo.Direction.FORWARD
							? Servo.Direction.REVERSE
							: Servo.Direction.FORWARD
			);
		}
		if (gamepad1.rightStickButtonWasPressed()) {
			current.servo.setDirection(Servo.Direction.FORWARD);
		}
		
		// === SWEEP TEST ===
		if (gamepad1.backWasPressed()) {
			isSweeping = !isSweeping;
			if (isSweeping) {
				commandedPosition = 0.0;
				sweepDirection = 1.0;
			}
		}
		
		if (isSweeping) {
			commandedPosition += sweepSpeed * sweepDirection;
			if (commandedPosition >= 1.0) {
				commandedPosition = 1.0;
				sweepDirection = -1.0;
			} else if (commandedPosition <= 0.0) {
				commandedPosition = 0.0;
				sweepDirection = 1.0;
			}
		}
		
		// === CLAMP AND APPLY POSITION ===
		commandedPosition = Math.max(0.0, Math.min(1.0, commandedPosition));
		current.servo.setPosition(commandedPosition);
		
		// === TELEMETRY ===
		displayTelemetry(current);
	}
	
	private void displayTelemetry(ServoInfo current) {
		logging.clearDynamic();
		
		// Header with servo selection
		logging.addLine("═══════ SERVO DEBUG ═══════");
		logging.addLine("");
		
		// Servo selector
		StringBuilder selector = new StringBuilder();
		for (int i = 0; i < servos.size(); i++) {
			if (i == selectedIndex) {
				selector.append("【").append(servos.get(i).name).append("】 ");
			} else {
				selector.append(servos.get(i).name).append(" ");
			}
		}
		logging.addLine(selector.toString());
		logging.addLine("");
		
		// Position info
		logging.addLine("═══════ POSITION ═══════");
		logging.addData("Commanded", "%.4f", commandedPosition);
		logging.addData("Reported", "%.4f", current.servo.getPosition());
		
		// Visual position bar
		int barLength = 20;
		int filledLength = (int) (commandedPosition * barLength);
		StringBuilder bar = new StringBuilder("[");
		for (int i = 0; i < barLength; i++) {
			if (i == filledLength) {
				bar.append("●");
			} else if (i < filledLength) {
				bar.append("═");
			} else {
				bar.append("─");
			}
		}
		bar.append("]");
		logging.addLine(bar.toString());
		logging.addLine("");
		
		// Servo details
		logging.addLine("═══════ DETAILS ═══════");
		logging.addData("Direction", current.servo.getDirection());
		logging.addData("PWM Enabled", isPwmEnabled ? "✓ YES" : "✗ NO");
		
		// Extended info if available
		if (current.servo instanceof ServoImplEx) {
			ServoImplEx servoEx = (ServoImplEx) current.servo;
			PwmControl.PwmRange range = servoEx.getPwmRange();
			logging.addData("PWM Range", "%.0f - %.0f µs", range.usPulseLower, range.usPulseUpper);
			
			// Calculate approximate PWM for current position
			double pwmUs = range.usPulseLower + (range.usPulseUpper - range.usPulseLower) * commandedPosition;
			logging.addData("Est. PWM", "%.0f µs", pwmUs);
		}
		
		// Controller info
		logging.addData("Port", current.portNumber);
		if (current.controllerName != null) {
			logging.addData("Controller", current.controllerName);
		}
		logging.addLine("");
		
		// Sweep status
		if (isSweeping) {
			logging.addLine("🔄 SWEEP MODE ACTIVE");
			logging.addLine("");
		}
		
		// Controls reminder
		logging.addLine("═══════ CONTROLS ═══════");
		logging.addLine("←→ Select | ↑↓ ±0.05 | LR Bump ±0.01");
		logging.addLine("Triggers ±0.001 | A/B/Y = 0/0.5/1");
		logging.addLine("X=PWM | L3=Flip | BACK=Sweep");
		
		logging.update();
	}
	
	/**
	 * Discovers all servos in the hardware map.
	 */
	private List<ServoInfo> discoverServos() {
		List<ServoInfo> discovered = new ArrayList<>();
		
		for (Servo servo : hardwareMap.getAll(Servo.class)) {
			ServoInfo info = new ServoInfo();
			info.servo = servo;
			info.name = hardwareMap.getNamesOf(servo).iterator().next();
			
			// Try to get extended info
			if (servo instanceof ServoImplEx) {
				ServoImplEx servoEx = (ServoImplEx) servo;
				ServoControllerEx controller = (ServoControllerEx) servoEx.getController();
				info.portNumber = servoEx.getPortNumber();
				info.controllerName = hardwareMap.getNamesOf(controller).iterator().hasNext()
						? hardwareMap.getNamesOf(controller).iterator().next()
						: "Unknown";
			}
			
			discovered.add(info);
		}
		
		return discovered;
	}
	
	/**
	 * Container for servo information.
	 */
	private static class ServoInfo {
		Servo servo;
		String name;
		int portNumber = -1;
		String controllerName;
	}
}

