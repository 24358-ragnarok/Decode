package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

// Note: Ensure your Limelight 3a is configured via its web interface to use the AprilTag pipeline
// and is named "limelight" in your robot's hardware configuration.

@TeleOp(name = "Test: limelight", group = "Tests")
public class LimelightAimingTest extends LinearOpMode {
	
	// Define the specific AprilTag ID we are looking for.
	private static final int TARGET_TAG_ID = 20;
	
	// Declare the Limelight hardware object.
	private Limelight3A limelight;
	
	@Override
	public void runOpMode() {
		telemetry.log().setCapacity(10);
		telemetry.log().add("Limelight AprilTag Test OpMode Ready");
		telemetry.update();
		
		// Initialize the Limelight hardware object. This should match the name in your
		// hardware configuration file on the Control Hub.
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		
		waitForStart();
		
		while (opModeIsActive()) {
			// Get the latest vision results from the Limelight.
			LLResult result = limelight.getLatestResult();
			
			// Check if the result is valid (i.e., the Limelight has new data).
			if (result != null && result.isValid()) {
				// Get the list of all detected AprilTags (fiducials).
				List<FiducialResult> fiducials = result.getFiducialResults();
				
				boolean tagFound = false;
				for (FiducialResult fiducial : fiducials) {
					// Check if the current fiducial's ID matches our target.
					if (fiducial.getFiducialId() == TARGET_TAG_ID) {
						tagFound = true;
						
						// Get the 3D pose of the AprilTag relative to the camera.
						// The Limelight documentation uses a pose format of
						// [tx, ty, tz, roll, pitch, yaw] in meters and degrees.
						// We use getTargetPoseCameraSpace() for this example.
						Pose3D pose = fiducial.getTargetPoseCameraSpace();
						
						double x = pose.getPosition().x;
						double y = pose.getPosition().y;
						double z = pose.getPosition().z; // z is distance in the camera's forward direction
						
						telemetry.addData("Status", "AprilTag %d Found!", TARGET_TAG_ID);
						telemetry.addData("Tag ID", fiducial.getFiducialId());
						telemetry.addData("X (degrees)", "%.2f", fiducial.getTargetXDegrees());
						telemetry.addData("Y (degrees)", "%.2f", fiducial.getTargetYDegrees());
						telemetry.addData("Area (%%)", "%.2f", fiducial.getTargetArea());
						
						telemetry.addLine();
						telemetry.addData("Pose Data (Camera Space)", "---");
						telemetry.addData("Distance (meters)", "%.2f", z);
						telemetry.addData("Pose X (meters)", "%.2f", x);
						telemetry.addData("Pose Y (meters)", "%.2f", y);
						
						// We've found the target, so no need to continue looping.
						break;
					}
				}
				
				if (!tagFound) {
					telemetry.addData("Status", "Looking for AprilTag %d...", TARGET_TAG_ID);
					telemetry.addData("Tags Seen", fiducials.size());
				}
			} else {
				telemetry.addData("Status", "Waiting for valid Limelight data...");
			}
			
			telemetry.update();
			sleep(20);
		}
	}
}