package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.BALL_INTAKE_WAIT_S;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.LAUNCH_STABILITY_WAIT_S;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.actions.CurvePathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.EndAtAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.EndPickupAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LaunchAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LinearPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.PickupBallAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.PrepareLaunchAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ScanAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SlowLinearPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SplinedPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.WaitAction;
import org.firstinspires.ftc.teamcode.configuration.MatchState;
import org.firstinspires.ftc.teamcode.configuration.Settings;

/**
 * Fluent builder for creating autonomous sequences.
 * Provides a clean, readable DSL for defining autonomous routines.
 * <p>
 * Example usage:
 *
 * <pre>
 * AutonomousSequence sequence = new SequenceBuilder(matchSettings)
 * 		.moveTo(Settings.Positions.Samples.Preset1.PREP)
 * 		.startIntake()
 * 		.moveSlowlyTo(Settings.Positions.Samples.Preset1.END)
 * 		.stopIntake()
 * 		.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
 * 				Settings.Positions.ControlPoints.FROM_PRESET1_TO_CLOSE, "Launch")
 * 		.launch()
 * 		.build();
 * </pre>
 */
public class SequenceBuilder {
	
	private final AutonomousSequence sequence;
	
	/**
	 * Creates a new sequence builder.
	 */
	public SequenceBuilder() {
		this.sequence = new AutonomousSequence();
	}
	
	/**
	 * Static factory method to create Far position sequences.
	 */
	public static AutonomousSequence buildFarSequence() {
		return new SequenceBuilder()
				.scanObelisk()
				// Launch preloads
				.prepLaunch()
				.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
				.wait(LAUNCH_STABILITY_WAIT_S)
				.launch()
				
				// Get ball set I
				.moveSplineTo(Settings.Positions.Samples.Preset1.PREP,
						"Prep Preset 1",
						Settings.Positions.ControlPoints.PRESET_1_APPROACH)
				.startPickup()
				.moveSlowlyTo(Settings.Positions.Samples.Preset1.GRAB_1, "Grab Preset 1 Ball 1")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset1.GRAB_2, "Grab Preset 1 Ball 2")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset1.END, "Grab Preset 1 Ball 3")
				.wait(BALL_INTAKE_WAIT_S)
				.endPickup()
				
				// Launch ball set I
				.prepLaunch()
				.moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT,
						Settings.Positions.ControlPoints.FROM_PRESET3_TO_FAR, "Launch Preset1")
				.wait(LAUNCH_STABILITY_WAIT_S)
				.launch()
				
				// Get ball set II
				.moveSplineTo(Settings.Positions.Samples.Preset2.PREP,
						"Prep Preset2",
						Settings.Positions.ControlPoints.PRESET_2_APPROACH)
				.startPickup()
				.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_1, "Grab1 Preset2")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_2, "Grab2 Preset2")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
				.wait(BALL_INTAKE_WAIT_S)
				.endPickup()
				
				// Launch ball set II
				.prepLaunch()
				.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preset2")
				.wait(LAUNCH_STABILITY_WAIT_S)
				.launch()
				
				// Park
				.endPickup()
				.moveTo(Settings.Positions.Park.FAR, "Park")
				.endAt(Settings.Positions.Park.FAR)
				.build();
	}
	
	/**
	 * Static factory method to create Close position sequences.
	 */
	public static AutonomousSequence buildCloseSequence() {
		return new SequenceBuilder()
				.moveTo(Settings.Positions.Towers.SCAN)
				.scanObelisk()
				// Launch preload
				.prepLaunch()
				.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
				.wait(LAUNCH_STABILITY_WAIT_S)
				.launch()
				
				// Get ball set I (Preset3 for close sequence)
				.moveTo(Settings.Positions.Samples.Preset3.PREP, "Prep Preset3")
				.startPickup()
				.moveSlowlyTo(Settings.Positions.Samples.Preset3.GRAB_1, "Grab1 Preset3")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset3.GRAB_2, "Grab2 Preset3")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset3.END, "End Preset3")
				.wait(BALL_INTAKE_WAIT_S)
				.endPickup()
				
				// Launch ball set I
				.prepLaunch()
				.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
						Settings.Positions.ControlPoints.FROM_PRESET3_TO_CLOSE, "Launch Preset3")
				.wait(LAUNCH_STABILITY_WAIT_S)
				.launch()
				
				// Get ball set II (Preset2 for close sequence)
				.moveTo(Settings.Positions.Samples.Preset2.PREP, "Prep Preset2")
				.startPickup()
				.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_1, "Grab1 Preset2")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_2, "Grab2 Preset2")
				.wait(BALL_INTAKE_WAIT_S)
				.moveSlowlyTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
				.wait(BALL_INTAKE_WAIT_S)
				.endPickup()
				
				// Launch ball set II
				.prepLaunch()
				.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
						Settings.Positions.ControlPoints.FROM_PRESET2_TO_CLOSE, "Launch Preset2")
				.wait(LAUNCH_STABILITY_WAIT_S)
				.launch()
				
				// Park
				.endPickup()
				.moveTo(Settings.Positions.Park.CLOSE, "Park")
				.endAt(Settings.Positions.Park.CLOSE)
				.build();
	}
	
	/**
	 * Adds a linear path action to a target pose.
	 *
	 * @param targetPose The target pose (in BLUE alliance coordinates)
	 * @param name       Human-readable name for telemetry
	 * @return this (for method chaining)
	 */
	public SequenceBuilder moveTo(Pose targetPose, String name) {
		sequence.addAction(new LinearPathAction(targetPose, name));
		return this;
	}
	
	/**
	 * Adds a linear path action to a target pose with auto-generated name.
	 */
	public SequenceBuilder moveTo(Pose targetPose) {
		sequence.addAction(new LinearPathAction(targetPose));
		return this;
	}
	
	/**
	 * Adds a slow linear path action to a target pose.
	 */
	public SequenceBuilder moveSlowlyTo(Pose targetPose, String name) {
		sequence.addAction(new SlowLinearPathAction(targetPose, name));
		return this;
	}
	
	/**
	 * Adds a slow linear path action to a target pose with auto-generated name.
	 */
	public SequenceBuilder moveSlowlyTo(Pose targetPose) {
		sequence.addAction(new SlowLinearPathAction(targetPose));
		return this;
	}
	
	/**
	 * Adds a splined path action to a target pose.
	 */
	public SequenceBuilder moveSplineTo(Pose targetPose, String name, Pose... controlPoints) {
		sequence.addAction(new SplinedPathAction(targetPose, name, controlPoints));
		return this;
	}
	
	/**
	 * Adds a splined path action with auto-generated control points.
	 */
	public SequenceBuilder moveSplineTo(Pose targetPose, String name) {
		sequence.addAction(new SplinedPathAction(targetPose, name));
		return this;
	}
	
	/**
	 * Adds a curve path action with control points.
	 */
	public SequenceBuilder moveCurveTo(Pose targetPose, String name, Pose... controlPoints) {
		sequence.addAction(new CurvePathAction(targetPose, name, controlPoints));
		return this;
	}
	
	/**
	 * Adds a curve path action with control points and auto-generated name.
	 */
	public SequenceBuilder moveCurveTo(Pose targetPose, Pose... controlPoints) {
		sequence.addAction(new CurvePathAction(targetPose, controlPoints));
		return this;
	}
	
	/**
	 * Convenience method for single control point curves.
	 */
	public SequenceBuilder moveCurveToVia(Pose targetPose, Pose controlPoint, String name) {
		sequence.addAction(CurvePathAction.withSingleControlPoint(targetPose, controlPoint, name));
		return this;
	}
	
	/**
	 * Adds an end-at action to hold position at a target pose.
	 */
	public SequenceBuilder endAt(Pose targetPose) {
		Pose finalPose = (MatchState.getAllianceColor() == MatchState.AllianceColor.BLUE)
				? targetPose
				: Settings.Field.mirrorPose(targetPose);
		sequence.addAction(new EndAtAction(finalPose));
		return this;
	}
	
	/**
	 * Adds an action to start the intake.
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder startPickup() {
		sequence.addAction(new PickupBallAction());
		return this;
	}
	
	/**
	 * Adds an action to scan the Motif obelisk.
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder scanObelisk() {
		sequence.addAction(new ScanAction());
		return this;
	}
	
	public SequenceBuilder prepLaunch() {
		sequence.addAction(new PrepareLaunchAction());
		return this;
	}
	
	/**
	 * Adds an action to stop the intake.
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder endPickup() {
		sequence.addAction(new EndPickupAction());
		return this;
	}
	
	/**
	 * Adds a launch action (waits for spindex to empty).
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder launch() {
		sequence.addAction(new LaunchAction());
		return this;
	}
	
	/**
	 * Adds a wait action.
	 *
	 * @param seconds Duration to wait in seconds
	 * @return this (for method chaining)
	 */
	public SequenceBuilder wait(double seconds) {
		sequence.addAction(new WaitAction(seconds));
		return this;
	}
	
	/**
	 * Adds a custom action.
	 *
	 * @param action The action to add
	 * @return this (for method chaining)
	 */
	public SequenceBuilder addAction(AutonomousAction action) {
		sequence.addAction(action);
		return this;
	}
	
	/**
	 * Adds multiple actions that run in parallel.
	 *
	 * @param actions The actions to run simultaneously
	 * @return this (for method chaining)
	 */
	public SequenceBuilder parallel(AutonomousAction... actions) {
		sequence.addAction(new ParallelAction(actions));
		return this;
	}
	
	/**
	 * Convenience method: Complete intake cycle (prep -> intake -> stop).
	 * Represents going to a sample, picking it up, and returning.
	 *
	 * @param prepPose Path to approach the sample
	 * @param endPose  Path to pick up the sample
	 * @return this (for method chaining)
	 */
	public SequenceBuilder intakeCycle(Pose prepPose, Pose endPose) {
		return this
				.moveTo(prepPose)
				.startPickup()
				.moveSlowlyTo(endPose)
				.endPickup();
	}
	
	/**
	 * Convenience method: Complete launch cycle (launch -> wait for empty -> move).
	 * Represents launching samples and then moving to next position.
	 *
	 * @param nextPose Path to follow after launching
	 * @return this (for method chaining)
	 */
	public SequenceBuilder launchAndMove(Pose nextPose) {
		return this
				.launch()
				.moveTo(nextPose);
	}
	
	/**
	 * Builds the sequence.
	 *
	 * @return The completed autonomous sequence
	 */
	public AutonomousSequence build() {
		return sequence;
	}
}
