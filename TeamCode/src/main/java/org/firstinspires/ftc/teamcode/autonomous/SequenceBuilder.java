package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Autonomous.BALL_INTAKE_WAIT_S;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LaunchAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.OverrideIntakeIn;
import org.firstinspires.ftc.teamcode.autonomous.actions.OverrideTransferState;
import org.firstinspires.ftc.teamcode.autonomous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.PrepareLaunchAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.StopIntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.WaitAction;
import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

/**
 * Fluent builder for creating autonomous sequences.
 * Provides a clean, readable DSL for defining autonomous routines.
 * <p>
 * Example usage:
 *
 * <pre>
 * AutonomousSequence sequence = new SequenceBuilder(pathRegistry)
 * 		.followPath(PathSegment.FAR_PRESET_1_PREP)
 * 		.startIntake()
 * 		.followPath(PathSegment.FAR_PRESET_1_END)
 * 		.stopIntake()
 * 		.launch()
 * 		.build();
 * </pre>
 */
public class SequenceBuilder {

	private final AutonomousSequence sequence;
	private final PathRegistry pathRegistry;

	/**
	 * Creates a new sequence builder.
	 *
	 * @param pathRegistry The path registry for retrieving paths
	 */
	public SequenceBuilder(PathRegistry pathRegistry) {
		this.sequence = new AutonomousSequence();
		this.pathRegistry = pathRegistry;
	}

	/**
	 * Static factory method to create Far position sequences.
	 */
	public static AutonomousSequence buildFarSequence(PathRegistry pathRegistry) {
		return new SequenceBuilder(pathRegistry)
				// Launch preloads
				.prepLaunch()
				.followPath(PathRegistry.PathSegment.FAR_LAUNCH_0)
				.wait(0.25)
				.launch()

				// Get ball set I
				.followPath(PathRegistry.PathSegment.FAR_PRESET_1_PREP)
				.startIntake()
				.followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_1_GRAB_1)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_1_GRAB_2)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_1_END)
				.wait(BALL_INTAKE_WAIT_S)
				.stopIntake()

				// Launch ball set I
				.prepLaunch()
				.followPath(PathRegistry.PathSegment.FAR_LAUNCH_1)
				.launch()
				
				// Get ball set II
				.startIntake()
				.followPath(PathRegistry.PathSegment.FAR_PRESET_2_PREP)
				.followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_2_GRAB_1)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_2_GRAB_2)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_2_END)
				.wait(BALL_INTAKE_WAIT_S)
				.stopIntake()
				
				// Launch ball set II
				.prepLaunch()
				.followPath(PathRegistry.PathSegment.FAR_LAUNCH_2)
				.launch()
				
				// Get ball set III
				/**
				 * .followPath(PathRegistry.PathSegment.FAR_PRESET_3_PREP)
				 * .followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_3_GRAB_1)
				 * .wait(BALL_INTAKE_WAIT_S)
				 * .followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_3_GRAB_2)
				 * .wait(BALL_INTAKE_WAIT_S)
				 * .followPathSlowly(PathRegistry.PathSegment.FAR_PRESET_3_END)
				 * .wait(BALL_INTAKE_WAIT_S)
				 *
				 * // Launch ball set III
				 * .prepLaunch()
				 * .followPath(PathRegistry.PathSegment.FAR_LAUNCH_3)
				 * .launch()
				 */
				
				// Park
				.followPath(PathRegistry.PathSegment.FAR_PARK)
				.stopIntake()
				.build();
	}
	
	/**
	 * Static factory method to create Close position sequences.
	 */
	public static AutonomousSequence buildCloseSequence(PathRegistry pathRegistry) {
		return new SequenceBuilder(pathRegistry)
				// Launch preload
				.prepLaunch()
				.followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_0)
				.wait(0.25)
				.launch()
				
				// Get ball set I
				.startIntake()
				.followPath(PathRegistry.PathSegment.CLOSE_PRESET_1_PREP)
				.followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_1_GRAB_1)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_1_GRAB_2)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_1_END)
				.wait(BALL_INTAKE_WAIT_S)
				.stopIntake()
				
				// Launch ball set I
				.prepLaunch()
				.followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_1)
				.launch()
				
				// Get ball set II
				.startIntake()
				.followPath(PathRegistry.PathSegment.CLOSE_PRESET_2_PREP)
				.followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_2_GRAB_1)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_2_GRAB_2)
				.wait(BALL_INTAKE_WAIT_S)
				.followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_2_END)
				.wait(BALL_INTAKE_WAIT_S)
				.stopIntake()
				
				// Launch ball set II
				.prepLaunch()
				.followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_2)
				.launch()
				
				// Get ball set III
				/**
				 * .followPath(PathRegistry.PathSegment.CLOSE_PRESET_3_PREP)
				 * .followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_3_GRAB_1)
				 * .wait(BALL_INTAKE_WAIT_S)
				 * .followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_3_GRAB_2)
				 * .wait(BALL_INTAKE_WAIT_S)
				 * .followPathSlowly(PathRegistry.PathSegment.CLOSE_PRESET_3_END)
				 * .wait(BALL_INTAKE_WAIT_S)
				 *
				 * // Launch ball set III
				 * .prepLaunch()
				 * .followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_3)
				 * .launch()
				 */
				
				// Park
				.followPath(PathRegistry.PathSegment.CLOSE_PARK)
				.stopIntake()
				.build();
	}
	
	/**
	 * Adds a path following action.
	 *
	 * @param segment The path segment to follow
	 * @return this (for method chaining)
	 */
	public SequenceBuilder followPath(PathRegistry.PathSegment segment) {
		PathChain path = pathRegistry.getPath(segment);
		sequence.addAction(new FollowPathAction(path, segment.name()));
		return this;
	}
	
	public SequenceBuilder followPathSlowly(PathRegistry.PathSegment segment) {
		PathChain path = pathRegistry.getPath(segment);
		sequence.addAction(new FollowPathAction(path, segment.name(), true));
		return this;
	}
	
	/**
	 * Adds an action to start the intake.
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder startIntake() {
		sequence.addAction(new IntakeAction());
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
	public SequenceBuilder stopIntake() {
		sequence.addAction(new StopIntakeAction());
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
	 * @param prepSegment Path to approach the sample
	 * @param endSegment  Path to pick up the sample
	 * @return this (for method chaining)
	 */
	public SequenceBuilder intakeCycle(PathRegistry.PathSegment prepSegment, PathRegistry.PathSegment endSegment) {
		return this
				.followPath(prepSegment)
				.startIntake()
				.followPath(endSegment)
				.stopIntake();
	}
	
	/**
	 * Convenience method: Complete launch cycle (launch -> wait for empty -> move).
	 * Represents launching samples and then moving to next position.
	 *
	 * @param nextSegment Path to follow after launching
	 * @return this (for method chaining)
	 */
	public SequenceBuilder launchAndMove(PathRegistry.PathSegment nextSegment) {
		return this
				.launch()
				.followPath(nextSegment);
	}
	
	/**
	 * Adds an override action to force all transfer slots to be full with the
	 * specified color.
	 * Useful for testing and debugging autonomous sequences.
	 *
	 * @param artifactColor The color to fill all slots with
	 * @return this (for method chaining)
	 */
	public SequenceBuilder overrideTransferState(MatchSettings.ArtifactColor artifactColor) {
		sequence.addAction(new OverrideTransferState(artifactColor));
		return this;
	}
	
	/**
	 * Adds an override action to force all transfer slots to be full with PURPLE
	 * artifacts.
	 * This is a convenience method for the most common test case.
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder overrideTransferStateFull() {
		sequence.addAction(new OverrideTransferState());
		return this;
	}
	
	
	/**
	 * Adds an override action to force both intake motor and transfer entrance
	 * wheel to start.
	 * Leaves intake running when complete
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder overrideIntakeIn() {
		sequence.addAction(new OverrideIntakeIn());
		return this;
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
