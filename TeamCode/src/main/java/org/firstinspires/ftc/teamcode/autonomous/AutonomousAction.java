package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.hardware.MechanismManager;

/**
 * Represents a single action in an autonomous sequence.
 * Actions are composable, reusable units that encapsulate robot behaviors.
 * <p>
 * This interface follows the Command pattern, allowing for:
 * - Easy testing of individual actions
 * - Clear separation of concerns
 * - Reusability across different autonomous modes
 */
public interface AutonomousAction {
	
	/**
	 * Called once when the action starts.
	 * Use this to initialize motors, set initial states, start timers, etc.
	 *
	 * @param mechanisms The robot's mechanism manager
	 */
	void initialize(MechanismManager mechanisms);
	
	/**
	 * Called repeatedly while the action is running.
	 * Use this to update mechanisms, check sensors, etc.
	 *
	 * @param mechanisms The robot's mechanism manager
	 * @return true if the action is complete, false if it should continue
	 */
	boolean execute(MechanismManager mechanisms);
	
	/**
	 * Called once when the action ends (either normally or when interrupted).
	 * Use this for cleanup: stop motors, reset states, etc.
	 *
	 * @param mechanisms  The robot's mechanism manager
	 * @param interrupted true if the action was interrupted, false if it completed
	 *                    normally
	 */
	void end(MechanismManager mechanisms, boolean interrupted);
	
	/**
	 * @return A human-readable name for this action (used for telemetry/debugging)
	 */
	default String getName() {
		return this.getClass().getSimpleName();
	}
}
