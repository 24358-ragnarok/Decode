package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Manages a sequence of AutonomousActions.
 * Provides a clean, declarative way to define autonomous routines.
 * <p>
 * Features:
 * - Sequential execution of actions
 * - Automatic state management
 * - Progress tracking for telemetry
 * - Graceful handling of action interruption
 */
public class AutonomousSequence {
	
	private final List<AutonomousAction> actions;
	private int currentActionIndex;
	private boolean initialized;
	
	/**
	 * Creates a new autonomous sequence.
	 */
	public AutonomousSequence() {
		this.actions = new ArrayList<>();
		this.currentActionIndex = 0;
		this.initialized = false;
	}
	
	/**
	 * Adds an action to the sequence.
	 *
	 * @param action The action to add
	 * @return this (for method chaining)
	 */
	public AutonomousSequence addAction(AutonomousAction action) {
		actions.add(action);
		return this;
	}
	
	/**
	 * Adds multiple actions to the sequence.
	 *
	 * @param actionsToAdd The actions to add
	 * @return this (for method chaining)
	 */
	public AutonomousSequence addActions(AutonomousAction... actionsToAdd) {
		actions.addAll(Arrays.asList(actionsToAdd));
		return this;
	}
	
	/**
	 * Starts the sequence. Must be called before update().
	 *
	 * @param mechanisms The mechanism manager
	 */
	public void start(MechanismManager mechanisms) {
		currentActionIndex = 0;
		initialized = false;
		
		if (!actions.isEmpty()) {
			actions.get(0).initialize(mechanisms);
			initialized = true;
		}
	}
	
	/**
	 * Updates the current action in the sequence.
	 * Call this repeatedly in your autonomous loop.
	 *
	 * @param mechanisms The mechanism manager
	 */
	public void update(MechanismManager mechanisms) {
		if (isComplete()) {
			return; // No more actions to run
		}
		
		AutonomousAction currentAction = actions.get(currentActionIndex);
		
		// Execute the current action
		boolean actionComplete = currentAction.execute(mechanisms);
		
		if (actionComplete) {
			// End the current action
			currentAction.end(mechanisms, false);
			
			// Move to the next action
			currentActionIndex++;
			
			// Initialize the next action if it exists
			if (currentActionIndex < actions.size()) {
				actions.get(currentActionIndex).initialize(mechanisms);
			}
		}
	}
	
	/**
	 * Stops the sequence immediately, ending the current action.
	 *
	 * @param mechanisms The mechanism manager
	 */
	public void stop(MechanismManager mechanisms) {
		if (!isComplete() && initialized) {
			actions.get(currentActionIndex).end(mechanisms, true);
		}
	}
	
	/**
	 * @return true if all actions in the sequence have completed
	 */
	public boolean isComplete() {
		return currentActionIndex >= actions.size();
	}
	
	/**
	 * @return the index of the current action (0-based)
	 */
	public int getCurrentActionIndex() {
		return currentActionIndex;
	}
	
	/**
	 * @return the total number of actions in the sequence
	 */
	public int getTotalActions() {
		return actions.size();
	}
	
	/**
	 * @return the name of the current action, or "Complete" if done
	 */
	public String getCurrentActionName() {
		if (isComplete()) {
			return "Complete";
		}
		return actions.get(currentActionIndex).getName();
	}
	
	/**
	 * @return progress as a percentage (0-100)
	 */
	public double getProgressPercent() {
		if (actions.isEmpty()) {
			return 100.0;
		}
		return (currentActionIndex * 100.0) / actions.size();
	}
}
