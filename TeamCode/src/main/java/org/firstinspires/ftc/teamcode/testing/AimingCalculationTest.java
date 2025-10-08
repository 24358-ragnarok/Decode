package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.Settings;

import java.util.List;

/**
 * Test OpMode to verify the angle-based aiming calculations.
 * Shows raw Limelight data and the calculated trajectory values.
 */
@TeleOp(name = "Test: Aiming Calculations", group = "Tests")
public class AimingCalculationTest extends LinearOpMode {
	
	// Define the specific AprilTag ID we are looking for.
	private static final int TARGET_TAG_ID = 20; // Blue alliance basket
	
	// Declare the Limelight hardware object.
	private Limelight3A limelight;
	
	@Override
	public void runOpMode() {
		telemetry.log().setCapacity(15);
		telemetry.log().add("Aiming Calculation Test Ready");
		telemetry.log().add("Point camera at AprilTag " + TARGET_TAG_ID);
		telemetry.update();
		
		// Initialize the Limelight hardware object.
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		
		waitForStart();
		limelight.pipelineSwitch(1);
		
		while (opModeIsActive()) {
			// Get the latest vision results from the Limelight.
			LLResult result = limelight.getLatestResult();
			
			// Check if the result is valid.
			if (!result.getFiducialResults().isEmpty()) {
				List<FiducialResult> fiducials = result.getFiducialResults();
				
				boolean tagFound = false;
				for (FiducialResult fiducial : fiducials) {
					// Check if the current fiducial's ID matches our target.
					if (fiducial.getFiducialId() == TARGET_TAG_ID) {
						tagFound = true;
						
						// ===== RAW LIMELIGHT DATA =====
						telemetry.addLine("=== RAW LIMELIGHT DATA ===");
						double tx = fiducial.getTargetXDegrees();
						double ty = fiducial.getTargetYDegrees();
						telemetry.addData("Target ID", fiducial.getFiducialId());
						telemetry.addData("TX (Yaw Offset)°", "%.2f", tx);
						telemetry.addData("TY (Vertical Angle)°", "%.2f", ty);
						telemetry.addData("Area %%", "%.2f", fiducial.getTargetArea());
						
						// ===== CONFIGURATION VALUES =====
						telemetry.addLine();
						telemetry.addLine("=== CONFIGURATION ===");
						telemetry.addData("Launcher Height", "%.1f in", Settings.Aiming.LAUNCHER_HEIGHT_INCHES);
						telemetry.addData("Goal/Tag Height", "%.1f in", Settings.Aiming.GOAL_HEIGHT_INCHES);
						telemetry.addData("Target Offset", "%.1f in", Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES);
						telemetry.addData("Wheel Speed", "%.0f RPM", Settings.Aiming.WHEEL_SPEED_RPM);
						
						// ===== CALCULATIONS =====
						telemetry.addLine();
						telemetry.addLine("=== CALCULATIONS ===");
						
						// Prevent division by zero
						double tyActual = ty;
						if (Math.abs(ty) < 0.5) {
							tyActual = 0.5;
							telemetry.addData("Warning", "TY too small, using 0.5°");
						}
						
						// Calculate horizontal distance
						double heightDiffToTag = Settings.Aiming.GOAL_HEIGHT_INCHES
								- Settings.Aiming.LAUNCHER_HEIGHT_INCHES;
						double tyRadians = Math.toRadians(tyActual);
						double horizontalDistance = heightDiffToTag / Math.tan(tyRadians);
						
						telemetry.addData("Height to Tag", "%.1f in", heightDiffToTag);
						telemetry.addData("Horizontal Distance", "%.1f in", horizontalDistance);
						
						// Validate distance
						if (horizontalDistance <= 0) {
							telemetry.addData("ERROR", "Negative distance! Check heights");
						} else if (horizontalDistance > 300) {
							telemetry.addData("WARNING", "Distance > 300 in (very far)");
						}
						
						// Calculate aim point
						double desiredHeight = Settings.Aiming.GOAL_HEIGHT_INCHES
								+ Settings.Aiming.TARGET_HEIGHT_OFFSET_INCHES;
						double heightToAimPoint = desiredHeight
								- Settings.Aiming.LAUNCHER_HEIGHT_INCHES;
						
						telemetry.addData("Target Height", "%.1f in", desiredHeight);
						telemetry.addData("Height to Aim Point", "%.1f in", heightToAimPoint);
						
						// Calculate launch angle
						double launchAngleRad = Math.atan2(heightToAimPoint, horizontalDistance);
						double launchAngleDeg = Math.toDegrees(launchAngleRad);
						
						telemetry.addLine();
						telemetry.addLine("=== AIMING SOLUTION ===");
						telemetry.addData("Yaw Correction", "%.2f°", tx);
						telemetry.addData("Launch Angle", "%.2f°", launchAngleDeg);
						
						// Calculate velocity
						double launchVelocity = Settings.Aiming.wheelRpmToVelocity(
								Settings.Aiming.WHEEL_SPEED_RPM);
						telemetry.addData("Launch Velocity", "%.1f in/s", launchVelocity);
						telemetry.addData("Required RPM", "%.0f", Settings.Aiming.WHEEL_SPEED_RPM);
						
						// Estimated flight time (simplified)
						double vx = launchVelocity * Math.cos(launchAngleRad);
						double flightTime = horizontalDistance / vx;
						telemetry.addData("Est. Flight Time", "%.2f sec", flightTime);
						
						// We've found the target, so no need to continue looping.
						break;
					}
				}
				
				if (!tagFound) {
					telemetry.addData("Status", "Looking for Tag %d...", TARGET_TAG_ID);
					telemetry.addData("Tags Seen", fiducials.size());
					if (!fiducials.isEmpty()) {
						telemetry.addData("Visible IDs", getVisibleIds(fiducials));
					}
				}
			} else {
				telemetry.addData("Status", "Waiting for Limelight data...");
			}
			
			telemetry.update();
			sleep(50); // Update 20 times per second
		}
	}
	
	/**
	 * Helper method to show which tags are visible
	 */
	private String getVisibleIds(List<FiducialResult> fiducials) {
		StringBuilder ids = new StringBuilder();
		for (int i = 0; i < fiducials.size(); i++) {
			if (i > 0)
				ids.append(", ");
			ids.append(fiducials.get(i).getFiducialId());
		}
		return ids.toString();
	}
}
