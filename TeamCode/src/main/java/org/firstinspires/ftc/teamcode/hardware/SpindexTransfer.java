package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.EJECT_EXIT_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.EXIT_OFFSET;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.EXIT_SERVO_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.EXIT_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.INTAKE_SERVO_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.INTAKE_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.RAPID_FIRE_COOLDOWN_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.SLOT_INTAKE_POSITIONS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Spindex.TOLERANCE;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.software.ColorRangefinder;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

/**
 * Controls a 3-slot rotating spindex using the Command Pattern.
 * <p>
 * This design delegates all complex actions to dedicated "Command" objects.
 * The Spindex's main loop simply executes the current command, making the
 * system
 * highly scalable, readable, and robust against interruptions.
 */
@Deprecated
public class SpindexTransfer extends Mechanism {
	// Hardware & Config (public for Command access)
	public final ServoImplEx spindexServo;
	public final ServoImplEx exitSealServo;
	public final ServoImplEx intakeSealServo;
	public final ColorRangefinder colorSensor;
	public final MatchSettings matchSettings;
	public final double[] slotIntakePositions = new double[3];
	public final double exitOffset;
	public final double intakeOffset;
	
	// State Variables (package-private for Command access)
	final MatchSettings.ArtifactColor[] slots = new MatchSettings.ArtifactColor[3];
	final Object lock = new Object();
	long lastActionTimeMs = 0;
	private volatile double commandedPosition;
	// --- Command Pattern Implementation ---
	private Command currentCommand = new Command.IdleCommand();
	
	public SpindexTransfer(ServoImplEx spindexServo, ServoImplEx exitSealServo, ServoImplEx intakeSealServo, ColorRangefinder spindexColorSensor,
	                       MatchSettings matchSettings) {
		this.spindexServo = spindexServo;
		this.exitSealServo = exitSealServo;
		this.intakeSealServo = intakeSealServo;
		this.colorSensor = spindexColorSensor;
		this.matchSettings = matchSettings;
		
		for (int i = 0; i < 3; i++) {
			slotIntakePositions[i] = wrapServo(SLOT_INTAKE_POSITIONS[i]);
		}
		this.exitOffset = wrapServo(EXIT_OFFSET);
		this.intakeOffset = wrapServo(this.exitOffset + 0.5); // Intake is 180 degrees from exit
		
		Arrays.fill(slots, MatchSettings.ArtifactColor.UNKNOWN);
		this.commandedPosition = wrapServo(spindexServo.getPosition());
	}
	
	private static double wrapServo(double v) {
		v %= 1.0;
		if (v < 0)
			v += 1.0;
		return v;
	}
	
	private static double circularDistance(double a, double b) {
		double d = Math.abs(a - b);
		return Math.min(d, 1.0 - d);
	}
	
	public void start() {
		closeBothSeals();
		rotateSlotToExit(0);
	}
	
	/**
	 * Called periodically. This is the heart of the command executor.
	 */
	public void update() {
		// Execute the current command.
		if (currentCommand != null) {
			currentCommand.execute();
			// If the command is finished, revert to the idle state.
			if (currentCommand.isFinished()) {
				currentCommand = new Command.IdleCommand();
			}
		}
		
		// Handle color sensor reading separately, as it's always active.
		MatchSettings.ArtifactColor color = colorSensor.getArtifactColor();
		if (color != MatchSettings.ArtifactColor.UNKNOWN) {
			slots[getSlotIndexAtSensor()] = color;
		}
	}
	
	public void stop() {
		rotateSlotToExit(0);
	}
	
	// Add this method to your Spindex class
	public void eject() {
		submitCommand(new Command.EjectCommand(this));
	}
	
	/**
	 * Submits a new command to the spindex, allowing for intelligent interruption.
	 */
	private void submitCommand(Command newCommand) {
		synchronized (lock) {
			// Give the current command a chance to handle the new one (e.g., retargeting).
			// If it can't, the new command simply replaces the old one.
			if (currentCommand == null || !currentCommand.handleNewCommand(newCommand)) {
				currentCommand = newCommand;
				currentCommand.init(); // Initialize the new command
			}
		}
	}
	
	public boolean isEmpty() {
		for (MatchSettings.ArtifactColor slot : slots) {
			if (slot != MatchSettings.ArtifactColor.UNKNOWN) {
				return false;
			}
		}
		return true;
	}
	
	private void rotateSlotToExit(int slotIndex) {
		double targetPos = getExitPositionForSlot(slotIndex);
		submitCommand(new Command.RotateCommand(this, targetPos));
	}
	
	private void rotateSlotToIntake(int slotIndex) {
		double targetPos = getIntakePositionForSlot(slotIndex);
		submitCommand(new Command.RotateCommand(this, targetPos));
	}
	
	/**
	 * Submits a command to run the full intake sequence: find an empty slot,
	 * rotate to it, open the seal, and wait for a new artifact.
	 *
	 * @return Returns true if an intake command was started, false if the spindex
	 * is full.
	 */
	public boolean prepareForIntake() {
		// First, check if there's an empty slot without locking, for a quick exit.
		boolean hasEmptySlot = false;
		// This read is not synchronized, but it's only for a preliminary check.
		// The command itself will perform a safe, synchronized check.
		for (MatchSettings.ArtifactColor slot : slots) {
			if (slot == MatchSettings.ArtifactColor.UNKNOWN) {
				hasEmptySlot = true;
				break;
			}
		}
		
		if (hasEmptySlot) {
			submitCommand(new Command.IntakeCommand(this));
			return true;
		}
		return false; // Spindex is full
	}
	
	public void rapidFireSequence() {
		List<Integer> targets = findOptimalShotOrder(matchSettings.nextThreeArtifactsNeeded());
		if (!targets.isEmpty()) {
			submitCommand(new Command.RapidFireCommand(this, new ArrayDeque<>(targets)));
		}
	}
	
	// --- Hardware and State Helpers ---
	
	public double getCurrentServoPosition() {
		return commandedPosition;
	}
	
	void setServoPosition(double pos) {
		commandedPosition = wrapServo(pos);
		spindexServo.setPosition(commandedPosition);
	}
	
	void openIntakeSeal() {
		intakeSealServo.setPosition(INTAKE_SERVO_OPEN_POSITION);
	}
	
	// Add this public method to your Spindex class
	public void rotateNextArtifactToExit() {
		MatchSettings.ArtifactColor needed = matchSettings.nextArtifactNeeded();
		if (needed == MatchSettings.ArtifactColor.UNKNOWN) {
			return; // Nothing to do
		}
		
		// Find the first slot containing the needed artifact
		for (int i = 0; i < slots.length; i++) {
			if (slots[i] == needed) {
				// Found it. Create and submit a command to rotate this slot to the exit.
				double targetPos = getExitPositionForSlot(i);
				submitCommand(new Command.RotateCommand(this, targetPos));
				return; // We only need to move the first one we find
			}
		}
		// If we get here, no matching artifact was found in the spindex.
	}
	
	public boolean isNextArtifactAtExit() {
		MatchSettings.ArtifactColor needed = matchSettings.nextArtifactNeeded();
		if (needed == MatchSettings.ArtifactColor.UNKNOWN) {
			return false;
		}
		
		// 1. Is the spindex currently idle?
		boolean isIdle = (currentCommand instanceof Command.IdleCommand || currentCommand.isFinished());
		if (!isIdle) {
			return false;
		}
		
		// 2. Is there a slot aligned at the exit?
		Integer slotAtExit = getSlotIndexAtExit(); // This now checks for alignment
		if (slotAtExit == null) {
			return false; // No slot is aligned within tolerance.
		}
		
		// 3. Does the aligned slot contain the correct artifact?
		return slots[slotAtExit] == needed;
	}
	
	void ejectBallAtExit() {
		Integer slot = getSlotIndexAtExit(); // Return type is now Integer
		
		// Only proceed if a slot is actually aligned at the exit
		if (slot != null) {
			exitSealServo.setPosition(EXIT_SERVO_OPEN_POSITION);
			slots[slot] = MatchSettings.ArtifactColor.UNKNOWN;
			lastActionTimeMs = System.currentTimeMillis();
		}
	}
	
	void closeBothSeals() {
		exitSealServo.setPosition(EXIT_SERVO_CLOSED_POSITION);
		intakeSealServo.setPosition(INTAKE_SERVO_CLOSED_POSITION);
	}
	
	public boolean isAtTarget(double targetPosition) {
		return circularDistance(commandedPosition, targetPosition) <= TOLERANCE;
	}
	
	public double getExitPositionForSlot(int slotIndex) {
		return wrapServo(slotIntakePositions[slotIndex] + exitOffset);
	}
	
	public double getIntakePositionForSlot(int slotIndex) {
		return wrapServo(slotIntakePositions[slotIndex] + intakeOffset);
	}
	
	public int getSlotIndexAtSensor() {
		return nearestSlotIndexTo(commandedPosition);
	}
	
	/**
	 * Finds the index of the slot currently aligned at the exit position.
	 *
	 * @return The index (0-2) of the aligned slot, or null if no slot is
	 * within tolerance of the exit position.
	 */
	public Integer getSlotIndexAtExit() {
		// Step 1: Find the index of the slot that is physically nearest to the exit.
		int nearestIndex = nearestSlotIndexTo(wrapServo(commandedPosition - exitOffset));
		
		// Step 2: Determine the exact target position for that slot to be at the exit.
		double targetPositionForSlot = getExitPositionForSlot(nearestIndex);
		
		// Step 3: Check if the spindex is actually at that target position.
		boolean isAligned = isAtTarget(targetPositionForSlot);
		
		// Step 4: Only return the index if it's truly aligned, otherwise return null.
		return isAligned ? nearestIndex : null;
	}
	
	// --- Unchanged Helper Methods ---
	private List<Integer> findOptimalShotOrder(List<MatchSettings.ArtifactColor> needed) {
		MatchSettings.ArtifactColor[] currentSlots = Arrays.copyOf(slots, slots.length);
		List<Integer> availableSlotsForNeededArtifacts = new ArrayList<>();
		for (MatchSettings.ArtifactColor neededColor : needed) {
			for (int i = 0; i < currentSlots.length; i++) {
				if (currentSlots[i] == neededColor) {
					availableSlotsForNeededArtifacts.add(i);
					currentSlots[i] = null;
					break;
				}
			}
		}
		if (availableSlotsForNeededArtifacts.isEmpty())
			return new ArrayList<>();
		availableSlotsForNeededArtifacts.sort((slotA, slotB) -> {
			double distA = circularDistance(commandedPosition, getExitPositionForSlot(slotA));
			double distB = circularDistance(commandedPosition, getExitPositionForSlot(slotB));
			return Double.compare(distA, distB);
		});
		return availableSlotsForNeededArtifacts;
	}
	
	private int nearestSlotIndexTo(double servoPos) {
		int best = 0;
		double bestDist = Double.POSITIVE_INFINITY;
		for (int i = 0; i < 3; i++) {
			double d = circularDistance(servoPos, slotIntakePositions[i]);
			if (d < bestDist) {
				bestDist = d;
				best = i;
			}
		}
		return best;
	}
}

/**
 * Defines the interface for a command that the Spindex can execute.
 * Each command is a self-contained state machine for a specific task.
 */
abstract class Command {
	protected SpindexTransfer spindex; // Reference to the spindex hardware
	
	public Command(SpindexTransfer spindex) {
		this.spindex = spindex;
	}
	
	public Command() {
	}
	
	/**
	 * Called once when the command is first started.
	 */
	public void init() {
	}
	
	/**
	 * Called on every loop of the Spindex's update method.
	 */
	public abstract void execute();
	
	/**
	 * Returns true when the command has finished its task.
	 */
	public abstract boolean isFinished();
	
	/**
	 * Allows a command to intelligently handle a new incoming command.
	 *
	 * @return true if the new command was handled (e.g., retargeted), false
	 * otherwise.
	 */
	public boolean handleNewCommand(Command newCommand) {
		return false;
	}
	
	// --- Concrete Command Implementations ---
	
	// Add this method inside the abstract Command class
	public Command then(Command nextCommand) {
		// Note: We're passing 'this.spindex' so the new command group has the spindex
		// reference.
		return new SequentialCommand(this.spindex, this, nextCommand);
	}
	
	/**
	 * A default command that does nothing.
	 */
	public static class IdleCommand extends Command {
		@Override
		public void execute() {
		}
		
		@Override
		public boolean isFinished() {
			return false;
		} // Never finishes
	}
	
	/**
	 * A command to rotate the spindex to a target position.
	 */
	public static class RotateCommand extends Command {
		private final Runnable onCompleteAction;
		private State state = State.SEALING;
		private double targetPosition;
		
		public RotateCommand(SpindexTransfer spindex, double targetPosition, Runnable onCompleteAction) {
			super(spindex);
			this.targetPosition = targetPosition;
			this.onCompleteAction = onCompleteAction;
		}
		
		public RotateCommand(SpindexTransfer spindex, double targetPosition) {
			this(spindex, targetPosition, null);
		}
		
		@Override
		public void init() {
			this.state = State.SEALING;
		}
		
		@Override
		public void execute() {
			switch (state) {
				case SEALING:
					spindex.closeBothSeals();
					// Wait for eject cooldown before moving.
					if (System.currentTimeMillis() - spindex.lastActionTimeMs > EJECT_EXIT_TIME_MS) {
						state = State.ROTATING;
					}
					break;
				case ROTATING:
					spindex.setServoPosition(targetPosition);
					if (spindex.isAtTarget(targetPosition)) {
						if (onCompleteAction != null) {
							onCompleteAction.run();
						}
						state = State.FINISHED;
					}
					break;
				case FINISHED:
					// Do nothing
					break;
			}
		}
		
		@Override
		public boolean isFinished() {
			return state == State.FINISHED;
		}
		
		@Override
		public boolean handleNewCommand(Command newCommand) {
			// If the new command is also a rotation, we can just update our target!
			if (newCommand instanceof RotateCommand) {
				RotateCommand newRotateCmd = (RotateCommand) newCommand;
				this.targetPosition = newRotateCmd.targetPosition;
				// If we were already rotating, continue. If sealing, the new target will be
				// used
				// when we transition to the ROTATING state. This is intelligent retargeting.
				return true;
			}
			return false;
		}
		
		private enum State {
			SEALING, ROTATING, FINISHED
		}
	}
	
	// Add this class alongside the other command implementations
	
	/**
	 * A command to execute the rapid-fire sequence.
	 */
	public static class RapidFireCommand extends Command {
		
		private final Deque<Integer> targetQueue;
		private State state = State.SEALING;
		
		public RapidFireCommand(SpindexTransfer spindex, Deque<Integer> targetQueue) {
			super(spindex);
			this.targetQueue = targetQueue;
		}
		
		@Override
		public void execute() {
			switch (state) {
				case SEALING:
					spindex.closeBothSeals();
					if (System.currentTimeMillis() - spindex.lastActionTimeMs > EJECT_EXIT_TIME_MS) {
						state = State.ROTATING;
					}
					break;
				case ROTATING:
					if (targetQueue.isEmpty()) {
						state = State.FINISHED;
						break;
					}
					double targetPos = spindex.getExitPositionForSlot(targetQueue.peek());
					spindex.setServoPosition(targetPos);
					if (spindex.isAtTarget(targetPos)) {
						state = State.FIRING;
					}
					break;
				case FIRING:
					// Wait for rapid-fire cooldown
					if (System.currentTimeMillis() - spindex.lastActionTimeMs > RAPID_FIRE_COOLDOWN_MS) {
						spindex.ejectBallAtExit(); // This also updates lastActionTimeMs
						targetQueue.poll(); // Fired, so remove from queue.
						state = State.ROTATING; // Go back to rotating for the next target
					}
					break;
				case FINISHED:
					break;
			}
		}
		
		// Add this class alongside the other command implementations
		
		@Override
		public boolean isFinished() {
			return state == State.FINISHED;
		}
		
		private enum State {
			SEALING, ROTATING, FIRING, FINISHED
		}
	}
	
	public static class EjectCommand extends Command {
		private State state = State.READY_TO_FIRE;
		
		public EjectCommand(SpindexTransfer spindex) {
			super(spindex);
		}
		
		@Override
		public void execute() {
			switch (state) {
				case READY_TO_FIRE:
					// Only fire if the correct artifact is at the exit AND cooldown has passed.
					if (spindex.isNextArtifactAtExit() &&
							(System.currentTimeMillis() - spindex.lastActionTimeMs > RAPID_FIRE_COOLDOWN_MS)) {
						spindex.ejectBallAtExit(); // Perform the action
					}
					// This command is "fire-and-forget". It finishes on the first cycle
					// whether it fired or not, preventing it from blocking other commands.
					state = State.FINISHED;
					break;
				case FINISHED:
					// Do nothing
					break;
			}
		}
		
		@Override
		public boolean isFinished() {
			return state == State.FINISHED;
		}
		
		private enum State {
			READY_TO_FIRE, FINISHED
		}
	}
	
	/**
	 * A command to find an empty slot, rotate it to the intake,
	 * open the seal, and wait to sense a new artifact.
	 */
	public static class IntakeCommand extends Command {
		private State state = State.START;
		private int targetSlot = -1;
		
		public IntakeCommand(SpindexTransfer spindex) {
			super(spindex);
		}
		
		@Override
		public void init() {
			// Reset the state machine every time the command starts
			this.state = State.START;
			this.targetSlot = -1;
		}
		
		@Override
		public void execute() {
			synchronized (spindex.lock) { // Synchronize to safely access slots array
				switch (state) {
					case START:
						// Step 1: Find the first available empty slot
						for (int i = 0; i < 3; i++) {
							if (spindex.slots[i] == MatchSettings.ArtifactColor.UNKNOWN) {
								targetSlot = i;
								break;
							}
						}
						// If we found a slot, proceed to rotate. Otherwise, we're done.
						if (targetSlot != -1) {
							state = State.SEALING;
						} else {
							state = State.FINISHED; // No empty slots
						}
						break;
					
					case SEALING:
						// Step 2: Ensure seals are closed and wait for any eject cooldown
						spindex.closeBothSeals();
						if (System.currentTimeMillis() - spindex.lastActionTimeMs > EJECT_EXIT_TIME_MS) {
							state = State.ROTATING;
						}
						break;
					
					case ROTATING:
						// Step 3: Rotate the target slot to the intake position
						double intakePos = spindex.getIntakePositionForSlot(targetSlot);
						spindex.setServoPosition(intakePos);
						if (spindex.isAtTarget(intakePos)) {
							state = State.INTAKING;
						}
						break;
					
					case INTAKING:
						// Step 4: Open the seal and wait for a color to be detected
						spindex.openIntakeSeal();
						MatchSettings.ArtifactColor detectedColor = spindex.colorSensor.getArtifactColor();
						if (detectedColor != MatchSettings.ArtifactColor.UNKNOWN) {
							spindex.slots[targetSlot] = detectedColor; // Safely update the slot
							state = State.FINISHING;
						}
						// Note: You could add a timeout here to prevent getting stuck
						break;
					
					case FINISHING:
						// Step 5: Close the seal and finish the command
						spindex.closeBothSeals();
						state = State.FINISHED;
						break;
					
					case FINISHED:
						// Do nothing
						break;
				}
			}
		}
		
		@Override
		public boolean isFinished() {
			return state == State.FINISHED;
		}
		
		private enum State {
			START, SEALING, ROTATING, INTAKING, FINISHING, FINISHED
		}
	}
	
	/**
	 * A command that runs a series of other commands in sequence.
	 */
	public static class SequentialCommand extends Command {
		private final Deque<Command> commandQueue = new ArrayDeque<>();
		private Command currentCommand;
		
		public SequentialCommand(SpindexTransfer spindex, Command... commands) {
			super(spindex);
			Collections.addAll(this.commandQueue, commands);
		}
		
		@Override
		public void execute() {
			// If there is no current command, get the next one from the queue
			if (currentCommand == null) {
				currentCommand = commandQueue.poll();
				if (currentCommand != null) {
					currentCommand.init(); // Initialize the new command
				}
			}
			
			// If we have a command to run, execute it
			if (currentCommand != null) {
				currentCommand.execute();
				// If it just finished, clear it so we can get the next one
				if (currentCommand.isFinished()) {
					currentCommand = null;
				}
			}
		}
		
		@Override
		public boolean isFinished() {
			// The sequence is finished when the queue is empty and no command is running
			return commandQueue.isEmpty() && currentCommand == null;
		}
	}
}