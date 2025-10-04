package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LaunchAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.StopIntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.WaitAction;

/**
 * Fluent builder for creating autonomous sequences.
 * Provides a clean, readable DSL for defining autonomous routines.
 * <p>
 * Example usage:
 *
 * <pre>
 * AutonomousSequence sequence = new SequenceBuilder(pathRegistry)
 *         .followPath(PathSegment.FAR_PRESET_1_PREP)
 *         .startIntake()
 *         .followPath(PathSegment.FAR_PRESET_1_END)
 *         .stopIntake()
 *         .launch()
 *         .build();
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
				// Cycle 1: First sample
				.launchAndMove(PathRegistry.PathSegment.FAR_PRESET_1_PREP)
				.intakeCycle(PathRegistry.PathSegment.FAR_PRESET_1_PREP, PathRegistry.PathSegment.FAR_PRESET_1_END)
				.followPath(PathRegistry.PathSegment.FAR_LAUNCH_1)
				
				// Cycle 2: Second sample
				.launchAndMove(PathRegistry.PathSegment.FAR_PRESET_2_PREP)
				.intakeCycle(PathRegistry.PathSegment.FAR_PRESET_2_PREP, PathRegistry.PathSegment.FAR_PRESET_2_END)
				.followPath(PathRegistry.PathSegment.FAR_LAUNCH_2)
				
				// Cycle 3: Third sample
				.launch()
				.startIntake()
				.followPath(PathRegistry.PathSegment.FAR_PRESET_3)
				.stopIntake()
				.followPath(PathRegistry.PathSegment.FAR_LAUNCH_3)
				
				// Final launch and park
				.launchAndMove(PathRegistry.PathSegment.FAR_PARK)
				.build();
	}
	
	/**
	 * Static factory method to create Close position sequences.
	 */
	public static AutonomousSequence buildCloseSequence(PathRegistry pathRegistry) {
		return new SequenceBuilder(pathRegistry)
				// Cycle 1: First sample
				.launchAndMove(PathRegistry.PathSegment.CLOSE_PRESET_1_PREP)
				.intakeCycle(PathRegistry.PathSegment.CLOSE_PRESET_1_PREP, PathRegistry.PathSegment.CLOSE_PRESET_1_END)
				.followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_1)
				
				// Cycle 2: Second sample
				.launchAndMove(PathRegistry.PathSegment.CLOSE_PRESET_2_PREP)
				.intakeCycle(PathRegistry.PathSegment.CLOSE_PRESET_2_PREP, PathRegistry.PathSegment.CLOSE_PRESET_2_END)
				.followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_2)
				
				// Cycle 3: Third sample
				.launchAndMove(PathRegistry.PathSegment.CLOSE_PRESET_3_PREP)
				.intakeCycle(PathRegistry.PathSegment.CLOSE_PRESET_3_PREP, PathRegistry.PathSegment.CLOSE_PRESET_3_END)
				.followPath(PathRegistry.PathSegment.CLOSE_LAUNCH_3)
				
				// Final launch and park
				.launchAndMove(PathRegistry.PathSegment.CLOSE_PARK)
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
	
	/**
	 * Adds an action to start the intake.
	 *
	 * @return this (for method chaining)
	 */
	public SequenceBuilder startIntake() {
		sequence.addAction(new IntakeAction());
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
	 * Convenience method: follow path and start intake simultaneously.
	 *
	 * @param segment The path segment to follow
	 * @return this (for method chaining)
	 */
	public SequenceBuilder followPathWithIntake(PathRegistry.PathSegment segment) {
		PathChain path = pathRegistry.getPath(segment);
		parallel(
				new FollowPathAction(path, segment.name()),
				new IntakeAction());
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
	 * Builds the sequence.
	 *
	 * @return The completed autonomous sequence
	 */
	public AutonomousSequence build() {
		return sequence;
	}
}
