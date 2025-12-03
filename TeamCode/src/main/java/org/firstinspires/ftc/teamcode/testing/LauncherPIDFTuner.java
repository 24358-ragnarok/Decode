package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.configuration.Settings;

/**
 * PIDF Tuning OpMode for Launcher Motors
 * <p>
 * This OpMode allows you to modify PIDF coefficients for both launcher motors
 * using the gamepad and apply the changes in real-time. Both motors are
 * controlled simultaneously to ensure consistent tuning.
 * <p>
 * Controls:
 * - DPAD Up/Down: Adjust P value
 * - DPAD Left/Right: Adjust I value
 * - Left Bumper/Right Bumper: Adjust D value
 * - Left Trigger/Right Trigger: Adjust F value
 * - A Button: Apply new PIDF values to both motors
 * - B Button: Reset to original PIDF values
 * - X/Y Buttons: Cycle through step size presets
 * - START Button: Toggle test mode (run motors at target velocity)
 * - Left Stick Y: Adjust target velocity in test mode
 * - BACK Button: Stop motors
 * - Right Stick Button: Undo last apply
 *
 * @author Created for launcher motor PIDF tuning
 */
@TeleOp(name = "Launcher PIDF Tuner", group = "Testing")
public class LauncherPIDFTuner extends LinearOpMode {
	
	private static final long STATUS_MESSAGE_DURATION_MS = 5000;
	private static final double[] STEP_SIZE_PRESETS = {0.001, 0.01, 0.1, 0.5, 1.0};
	// Motor references
	private DcMotorEx launcherRight;
	private DcMotorEx launcherLeft;
	// Current PIDF values being tuned
	private double currentP = 2.5;
	private double currentI = 0.1;
	private double currentD = 0.2;
	private double currentF = 0.5;
	// Adjustment step size
	private double stepSize = 0.01;
	// Original PIDF coefficients (stored for reset)
	private PIDFCoefficients pidfOrigRight;
	private PIDFCoefficients pidfOrigLeft;
	// Status message
	private String statusMessage = "";
	private long statusMessageTime = 0;
	// Edge detection for right stick button
	private boolean rightStickButtonPressed = false;
	// Motor test mode
	private boolean testModeActive = false;
	private double targetVelocity = 0.0;
	private int stepSizePresetIndex = 1; // Start at 0.01
	
	// Previous applied values for undo
	private PIDFCoefficients lastAppliedRight;
	private PIDFCoefficients lastAppliedLeft;
	
	@Override
	public void runOpMode() {
		// Get references to both launcher motors
		launcherRight = Settings.Hardware.LAUNCHER_RIGHT.fromHardwareMap(hardwareMap);
		launcherLeft = Settings.Hardware.LAUNCHER_LEFT.fromHardwareMap(hardwareMap);
		
		// Set motors to use encoder mode
		launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		// Store original PIDF coefficients
		pidfOrigRight = launcherRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
		pidfOrigLeft = launcherLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
		
		// Initialize current values from original (use average if they differ)
		currentP = (pidfOrigRight.p + pidfOrigLeft.p) / 2.0;
		currentI = (pidfOrigRight.i + pidfOrigLeft.i) / 2.0;
		currentD = (pidfOrigRight.d + pidfOrigLeft.d) / 2.0;
		currentF = (pidfOrigRight.f + pidfOrigLeft.f) / 2.0;
		
		// Initialize step size from preset
		stepSize = STEP_SIZE_PRESETS[stepSizePresetIndex];
		
		// Initialize last applied values
		lastAppliedRight = pidfOrigRight;
		lastAppliedLeft = pidfOrigLeft;
		
		// Display initial information
		telemetry.addLine("╔═══════════════════════════════════╗");
		telemetry.addLine("║   LAUNCHER PIDF TUNER            ║");
		telemetry.addLine("╚═══════════════════════════════════╝");
		telemetry.addLine();
		telemetry.addLine("📋 CONTROLS:");
		telemetry.addLine("  ↑↓ DPAD     → Adjust P");
		telemetry.addLine("  ←→ DPAD     → Adjust I");
		telemetry.addLine("  LB/RB       → Adjust D");
		telemetry.addLine("  LT/RT       → Adjust F");
		telemetry.addLine("  A           → Apply to both motors");
		telemetry.addLine("  B           → Reset to original");
		telemetry.addLine("  X/Y         → Change step size");
		telemetry.addLine();
		telemetry.addLine("📊 ORIGINAL VALUES:");
		telemetry.addLine(String.format("  Right: P=%.4f  I=%.4f  D=%.4f  F=%.4f",
				pidfOrigRight.p, pidfOrigRight.i, pidfOrigRight.d, pidfOrigRight.f));
		telemetry.addLine(String.format("  Left:  P=%.4f  I=%.4f  D=%.4f  F=%.4f",
				pidfOrigLeft.p, pidfOrigLeft.i, pidfOrigLeft.d, pidfOrigLeft.f));
		telemetry.update();
		
		// Wait for start
		waitForStart();
		
		// Main loop
		while (opModeIsActive()) {
			// Handle gamepad input for adjusting PIDF values using wasPressed() for edge
			// detection
			// Adjust P value with DPAD Up/Down
			if (gamepad1.dpadUpWasPressed()) {
				currentP += stepSize;
				setStatusMessage(String.format("P increased: %.4f", currentP));
			}
			if (gamepad1.dpadDownWasPressed()) {
				currentP -= stepSize;
				currentP = Math.max(0, currentP); // Prevent negative values
				setStatusMessage(String.format("P decreased: %.4f", currentP));
			}
			
			// Adjust I value with DPAD Left/Right
			if (gamepad1.dpadLeftWasPressed()) {
				currentI -= stepSize;
				currentI = Math.max(0, currentI); // Prevent negative values
				setStatusMessage(String.format("I decreased: %.4f", currentI));
			}
			if (gamepad1.dpadRightWasPressed()) {
				currentI += stepSize;
				setStatusMessage(String.format("I increased: %.4f", currentI));
			}
			
			// Adjust D value with Left/Right Bumpers
			if (gamepad1.leftBumperWasPressed()) {
				currentD -= stepSize;
				currentD = Math.max(0, currentD); // Prevent negative values
				setStatusMessage(String.format("D decreased: %.4f", currentD));
			}
			if (gamepad1.rightBumperWasPressed()) {
				currentD += stepSize;
				setStatusMessage(String.format("D increased: %.4f", currentD));
			}
			
			// Adjust F value with Left/Right Triggers
			if (gamepad1.leftBumperWasPressed()) {
				currentF -= stepSize;
				currentF = Math.max(0, currentF); // Prevent negative values
				setStatusMessage(String.format("F decreased: %.4f", currentF));
			}
			
			if (gamepad1.rightBumperWasPressed()) {
				currentF += stepSize;
				setStatusMessage(String.format("F increased: %.4f", currentF));
			}
			
			// Adjust step size with X/Y buttons (cycle through presets)
			if (gamepad1.xWasPressed()) {
				stepSizePresetIndex = (stepSizePresetIndex + 1) % STEP_SIZE_PRESETS.length;
				stepSize = STEP_SIZE_PRESETS[stepSizePresetIndex];
				setStatusMessage(String.format("Step size: %.4f", stepSize));
			}
			if (gamepad1.yWasPressed()) {
				stepSizePresetIndex = (stepSizePresetIndex - 1 + STEP_SIZE_PRESETS.length) % STEP_SIZE_PRESETS.length;
				stepSize = STEP_SIZE_PRESETS[stepSizePresetIndex];
				setStatusMessage(String.format("Step size: %.4f", stepSize));
			}
			
			// Toggle test mode with D-PAD center (if supported) or use Start button
			if (gamepad1.startWasPressed()) {
				testModeActive = !testModeActive;
				if (testModeActive) {
					targetVelocity = 3000.0; // Default test velocity
					launcherRight.setVelocity(targetVelocity);
					launcherLeft.setVelocity(targetVelocity);
					setStatusMessage("🧪 Test mode ON - Velocity: " + (int) targetVelocity);
				} else {
					launcherRight.setPower(0);
					launcherLeft.setPower(0);
					setStatusMessage("⏹️  Test mode OFF");
				}
			}
			
			// Adjust test velocity with left stick Y (inverted, up = faster)
			if (testModeActive) {
				double stickInput = -gamepad1.left_stick_y; // Invert so up is positive
				if (Math.abs(stickInput) > 0.1) {
					targetVelocity += stickInput * 50.0;
					targetVelocity = Math.max(0, Math.min(6000, targetVelocity)); // Clamp to motor range
					launcherRight.setVelocity(targetVelocity);
					launcherLeft.setVelocity(targetVelocity);
				}
			}
			
			// Stop motors with Back button
			if (gamepad1.backWasPressed()) {
				testModeActive = false;
				launcherRight.setPower(0);
				launcherLeft.setPower(0);
				setStatusMessage("⏹️  Motors stopped");
			}
			
			// Apply new PIDF values when A is pressed
			if (gamepad1.aWasPressed()) {
				// Store previous values for undo
				lastAppliedRight = launcherRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
				lastAppliedLeft = launcherLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
				
				PIDFCoefficients pidfCurrent = new PIDFCoefficients(currentP, currentI, currentD, currentF);
				launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCurrent);
				launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCurrent);
				setStatusMessage("✅ Applied to both motors!");
			}
			
			// Undo last apply with Right Stick button (edge detection)
			if (gamepad1.right_stick_button && !rightStickButtonPressed) {
				rightStickButtonPressed = true;
				launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastAppliedRight);
				launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastAppliedLeft);
				currentP = (lastAppliedRight.p + lastAppliedLeft.p) / 2.0;
				currentI = (lastAppliedRight.i + lastAppliedLeft.i) / 2.0;
				currentD = (lastAppliedRight.d + lastAppliedLeft.d) / 2.0;
				currentF = (lastAppliedRight.f + lastAppliedLeft.f) / 2.0;
				setStatusMessage("↩️  Undone - Restored previous values");
			} else if (!gamepad1.right_stick_button) {
				rightStickButtonPressed = false;
			}
			
			// Reset to original PIDF values when B is pressed
			if (gamepad1.bWasPressed()) {
				launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfOrigRight);
				launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfOrigLeft);
				currentP = (pidfOrigRight.p + pidfOrigLeft.p) / 2.0;
				currentI = (pidfOrigRight.i + pidfOrigLeft.i) / 2.0;
				currentD = (pidfOrigRight.d + pidfOrigLeft.d) / 2.0;
				currentF = (pidfOrigRight.f + pidfOrigLeft.f) / 2.0;
				setStatusMessage("🔄 Reset to original values");
			}
			
			// Read back current PIDF coefficients from motors
			PIDFCoefficients pidfReadRight = launcherRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
			PIDFCoefficients pidfReadLeft = launcherLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
			
			// Get motor velocities
			double velocityRight = launcherRight.getVelocity();
			double velocityLeft = launcherLeft.getVelocity();
			double avgVelocity = (velocityRight + velocityLeft) / 2.0;
			
			// Calculate differences from original
			double avgOrigP = (pidfOrigRight.p + pidfOrigLeft.p) / 2.0;
			double avgOrigI = (pidfOrigRight.i + pidfOrigLeft.i) / 2.0;
			double avgOrigD = (pidfOrigRight.d + pidfOrigLeft.d) / 2.0;
			double avgOrigF = (pidfOrigRight.f + pidfOrigLeft.f) / 2.0;
			
			// Display telemetry with improved UI/UX
			telemetry.addLine("╔═══════════════════════════════════════╗");
			telemetry.addLine("║   LAUNCHER PIDF TUNER                 ║");
			telemetry.addLine("╚═══════════════════════════════════════╝");
			telemetry.addLine();
			
			// Status message
			if (System.currentTimeMillis() - statusMessageTime < STATUS_MESSAGE_DURATION_MS) {
				telemetry.addLine(statusMessage);
				telemetry.addLine();
			}
			
			// Motor performance (most important for tuning)
			telemetry.addLine("📊 MOTOR PERFORMANCE:");
			if (testModeActive) {
				telemetry.addLine(String.format("  Target: %.0f ticks/s", targetVelocity));
				telemetry.addLine(String.format("  Right:  %.0f ticks/s (Error: %.0f)",
						velocityRight, targetVelocity - velocityRight));
				telemetry.addLine(String.format("  Left:   %.0f ticks/s (Error: %.0f)",
						velocityLeft, targetVelocity - velocityLeft));
				telemetry.addLine(String.format("  Avg:    %.0f ticks/s | Diff: %.0f",
						avgVelocity, Math.abs(velocityRight - velocityLeft)));
			} else {
				telemetry.addLine("  Right:  " + String.format("%.0f", velocityRight) + " ticks/s");
				telemetry.addLine("  Left:   " + String.format("%.0f", velocityLeft) + " ticks/s");
				telemetry.addLine("  (Press START to enable test mode)");
			}
			telemetry.addLine();
			
			// Current values being tuned (compact)
			telemetry.addLine("✏️  TUNING VALUES:");
			telemetry.addLine(String.format("  P: %7.4f  I: %7.4f  D: %7.4f  F: %7.4f",
					currentP, currentI, currentD, currentF));
			boolean hasChanges = Math.abs(currentP - avgOrigP) > 0.0001 ||
					Math.abs(currentI - avgOrigI) > 0.0001 ||
					Math.abs(currentD - avgOrigD) > 0.0001 ||
					Math.abs(currentF - avgOrigF) > 0.0001;
			if (hasChanges) {
				telemetry.addLine("  ⚠️  Changes pending (Press A to apply)");
			}
			telemetry.addLine();
			
			// Applied values (compact, only show if different from tuning values)
			boolean valuesMatch = Math.abs(pidfReadRight.p - currentP) < 0.0001 &&
					Math.abs(pidfReadRight.i - currentI) < 0.0001 &&
					Math.abs(pidfReadRight.d - currentD) < 0.0001 &&
					Math.abs(pidfReadRight.f - currentF) < 0.0001;
			if (!valuesMatch) {
				telemetry.addLine("🔧 APPLIED (Right/Left):");
				telemetry.addLine(String.format("  P: %.4f/%.4f  I: %.4f/%.4f",
						pidfReadRight.p, pidfReadLeft.p, pidfReadRight.i, pidfReadLeft.i));
				telemetry.addLine(String.format("  D: %.4f/%.4f  F: %.4f/%.4f",
						pidfReadRight.d, pidfReadLeft.d, pidfReadRight.f, pidfReadLeft.f));
				telemetry.addLine();
			}
			
			// Compact controls reminder
			telemetry.addLine("🎮 CONTROLS:");
			telemetry.addLine("  ↑↓=P  ←→=I  LB/RB=D  LT/RT=F");
			telemetry.addLine("  A=Apply  B=Reset  X/Y=Step  START=Test  RSB=Undo");
			telemetry.addLine("  Step: " + String.format("%.4f", stepSize));
			telemetry.update();
			
			// Small delay to prevent excessive updates
			sleep(20);
		}
	}
	
	/**
	 * Sets a status message that will be displayed for a limited time
	 */
	private void setStatusMessage(String message) {
		statusMessage = message;
		statusMessageTime = System.currentTimeMillis();
	}
	
	/**
	 * Returns a visual indicator showing if a value has changed from original
	 */
	private String getChangeIndicator(double current, double original) {
		double diff = current - original;
		if (Math.abs(diff) < 0.0001) {
			return ""; // No change
		} else if (diff > 0) {
			return String.format("(+%.4f)", diff);
		} else {
			return String.format("(%.4f)", diff);
		}
	}
}
