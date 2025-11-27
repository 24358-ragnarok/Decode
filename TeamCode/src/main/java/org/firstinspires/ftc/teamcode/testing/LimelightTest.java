package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.software.LimelightManager;
import org.firstinspires.ftc.teamcode.software.game.Artifact;
import org.firstinspires.ftc.teamcode.software.game.Classifier;
import org.firstinspires.ftc.teamcode.software.game.Motif;

@TeleOp(name = "Test: Limelight", group = "Tests")
public class LimelightTest extends OpMode {
	private LimelightManager limelightManager;
	private LLResult latestResult;
	private Motif lastDetectedMotif = Motif.UNKNOWN;
	private Classifier lastClassifier = null;
	private boolean greenDetected = false;
	private boolean purpleDetected = false;
	
	@Override
	public void init() {
		limelightManager = new LimelightManager(Settings.Hardware.LIMELIGHT.fromHardwareMap(hardwareMap));
		telemetry.addLine("✅ Limelight initialized");
		telemetry.update();
	}
	
	@Override
	public void loop() {
		// Update limelight
		limelightManager.update();
		latestResult = limelightManager.limelight.getLatestResult();
		
		// Command controls
		if (gamepad1.aWasPressed()) {
			lastDetectedMotif = limelightManager.detectMotif();
		}
		if (gamepad1.xWasPressed()) {
			greenDetected = limelightManager.detectArtifact(Artifact.Color.GREEN);
		}
		if (gamepad1.bWasPressed()) {
			purpleDetected = limelightManager.detectArtifact(Artifact.Color.PURPLE);
		}
		if (gamepad1.yWasPressed()) {
			telemetry.addData("ok", "ok");
			lastClassifier = limelightManager.coerceClassifierState(lastDetectedMotif);
			MatchState.setClassifier(lastClassifier);
		}
		if (gamepad1.right_bumper) {
			latestResult = limelightManager.detectGoal();
		}
		
		// Display telemetry
		displayTelemetry();
	}
	
	private void displayTelemetry() {
		telemetry.addLine("=== LIMELIGHT TEST ===");
		telemetry.addLine();
		
		// Current pipeline
		telemetry.addData("Current Pipeline", limelightManager.getCurrentPipeline());
		telemetry.addLine();
		
		// Basic result stats
		if (latestResult != null) {
			telemetry.addData("Tx", "%.2f", latestResult.getTx());
			telemetry.addData("Ty", "%.2f", latestResult.getTy());
			telemetry.addData("Ta", "%.2f", latestResult.getTa());
			telemetry.addLine();
			
			// AprilTag results
			if (latestResult.getFiducialResults() != null && !latestResult.getFiducialResults().isEmpty()) {
				telemetry.addLine("AprilTags Detected:");
				for (LLResultTypes.FiducialResult fid : latestResult.getFiducialResults()) {
					telemetry.addData("  Tag ID", fid.getFiducialId());
					telemetry.addData("  X pixels", "%.2f", fid.getTargetXPixels());
					telemetry.addData("  Y pixels", "%.2f", fid.getTargetYPixels());
					Motif motif = Motif.fromApriltag(fid.getFiducialId());
					telemetry.addData("  Motif", motif);
				}
				telemetry.addLine();
			}
			
			// Color detection results
			if (latestResult.getColorResults() != null && !latestResult.getColorResults().isEmpty()) {
				telemetry.addLine("Color Targets Detected: " + latestResult.getColorResults().size());
				for (int i = 0; i < Math.min(latestResult.getColorResults().size(), 5); i++) {
					LLResultTypes.ColorResult color = latestResult.getColorResults().get(i);
					telemetry.addData("  Target " + i + " X", "%.0f px", color.getTargetXPixels());
					telemetry.addData("  Target " + i + " Y", "%.0f px", color.getTargetYPixels());
				}
				telemetry.addLine();
			}
		}
		
		// Detection results
		telemetry.addLine("=== DETECTION RESULTS ===");
		telemetry.addData("Last Motif", lastDetectedMotif);
		telemetry.addData("Green Detected", greenDetected);
		telemetry.addData("Purple Detected", purpleDetected);
		if (lastClassifier != null) {
			telemetry.addData("Classifier Motif", lastClassifier.getMotif());
			telemetry.addData("Classifier Ball Count", lastClassifier.getBallCount());
			telemetry.addData("Next Artifact Needed", MatchState.nextArtifactNeeded());
		}
		telemetry.addLine();
		
		// Controls
		telemetry.addLine("=== CONTROLS ===");
		telemetry.addLine("A:          Detect Motif");
		telemetry.addLine("X:          Detect Green Artifact");
		telemetry.addLine("B:          Detect Purple Artifact");
		telemetry.addLine("Y:          Coerce Classifier State");
		telemetry.addLine("RB:         Detect Goal");
		
		telemetry.update();
	}
	
	@Override
	public void stop() {
		if (limelightManager != null) {
			limelightManager.stop();
		}
	}
}

