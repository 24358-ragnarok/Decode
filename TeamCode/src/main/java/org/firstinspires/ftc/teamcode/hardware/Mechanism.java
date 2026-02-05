package org.firstinspires.ftc.teamcode.hardware;

/**
 * Abstract base class for all robot mechanisms.
 * Provides a standard lifecycle interface that all mechanisms must implement.
 * <p>
 * The lifecycle methods are called in the following order:
 * 1. start() - Called once when the robot BEGINS running (not On Init Pressed)
 * 2. update() - Called repeatedly during robot operation
 * 3. stop() - Called once when the robot is stopped
 */
@SuppressWarnings("ClassWithoutConstructor")
public abstract class Mechanism {
	
	/**
	 * Physically initializes the mechanism. Called once during robot startup.
	 * Use this method to configure hardware, set initial states, and prepare the mechanism for operation.
	 */
	public abstract void start();
	
	/**
	 * Updates the mechanism state. Called repeatedly during robot operation.
	 * Use this method for periodic tasks like sensor reading, state machine updates, and control loops.
	 */
	public abstract void update();
	
	/**
	 * Stops the mechanism safely. Called once when the robot is shutting down.
	 * Use this method to stop motors, reset states, and perform cleanup operations.
	 */
	public abstract void stop();
}
