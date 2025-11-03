package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

/**
 * Interface between the Limelight camera and the robot, providing vision processing
 * capabilities for artifact detection, AprilTag recognition, and goal targeting.
 * <p>
 * Manages pipeline switching between different vision modes:
 * - AprilTag detection for motif recognition and goal targeting
 * - Green artifact detection
 * - Purple artifact detection
 * <p>
 * Optimized to minimize overhead by avoiding unnecessary pipeline switches and
 * providing efficient result caching.
 */
public class LimelightManager extends Mechanism {
	public final Limelight3A limelight;
	public final MatchSettings matchSettings;
	LLResult currentResult;
	/**
	 * Current active pipeline - starts with AprilTag to detect obelisk during auto start
	 */
	Pipeline currentPipeline = Pipeline.APRILTAG;
	
	public LimelightManager(Limelight3A limelight, MatchSettings matchSettings) {
		this.limelight = limelight;
		this.matchSettings = matchSettings;
		start(); // limelight is a non-physical system and thus can be initialized at any time
	}
	
	/**
	 * Initializes limelight with polling rate of 100 Hz.
	 * Sets the current pipeline and starts the camera streaming.
	 */
	public void start() {
		setCurrentPipeline(currentPipeline);
		limelight.start();
		limelight.setPollRateHz(100);
	}
	
	/**
	 * Updates the limelight result cache. 
	 * Note: This causes significant overhead and should be avoided in tight loops.
	 */
	public void update() {
		limelight.getLatestResult();
	}
	
	public void stop() {
		limelight.stop();
	}
	
	/**
	 * Updates the data and checks if there is a desired object detected
	 *
	 * @return if an artifact is detected
	 */
	public boolean detectArtifact(MatchSettings.ArtifactColor color) {
		setCurrentPipeline(getPipelineFromColor(color));
		currentResult = limelight.getLatestResult();
		return currentResult.getTx() != 0 && currentResult.getTy() != 0
				&& (Math.abs(currentResult.getTx()) < Settings.Vision.LL_WINDOW_SIZE_DEGREES);
	}
	
	public final Pipeline getPipelineFromColor(MatchSettings.ArtifactColor color) {
		if (color == MatchSettings.ArtifactColor.GREEN) {
			return Pipeline.GREEN;
		} else {
			return Pipeline.PURPLE;
		}
	}
	
	public MatchSettings.Motif detectMotif() {
		setCurrentPipeline(Pipeline.APRILTAG);
		LLResult result = limelight.getLatestResult();
		if (result.getFiducialResults().isEmpty()) {
			return MatchSettings.Motif.UNKNOWN;
		}
		
		switch (result.getFiducialResults().get(0).getFiducialId()) {
			case 21:
				return MatchSettings.Motif.GPP;
			case 22:
				return MatchSettings.Motif.PGP;
			case 23:
				return MatchSettings.Motif.PPG;
			default:
				return MatchSettings.Motif.UNKNOWN;
		}
	}
	
	public LLResult detectGoal() {
		setCurrentPipeline(Pipeline.APRILTAG);
		return limelight.getLatestResult();
	}
	
	/**
	 * Switches the current pipeline to a new pipeline
	 *
	 * @param newPipeline The new pipeline to switch to:
	 *                    APRILTAG (1), GREEN (2), PURPLE (3)
	 */
	public void setCurrentPipeline(Pipeline newPipeline) {
		if (newPipeline == currentPipeline) {
			return;
		}
		currentPipeline = newPipeline;
		limelight.pipelineSwitch(currentPipeline.ordinal() + 1);
	}
	
	public enum Pipeline {
		APRILTAG, // pipe 1
		GREEN, // pipe 2
		PURPLE, // pipe 3
		UNKNOWN
	}
}
