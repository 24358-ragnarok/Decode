package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.BLIND_WINDOW_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.ENTRANCE_OPEN_DURATION_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.ENTRANCE_WHEEL_HOLD_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.ENTRANCE_WHEEL_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_FIRE_DURATION_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_WHEEL_FIRE_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_WHEEL_HOLD_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.MAX_CAPACITY;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_WHEEL_FORWARD_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_WHEEL_REVERSE_POWER;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

import java.util.Arrays;

/**
 * Transfer subsystem with positional awareness and CR wheel management.
 * <p>
 * Positions: index 0 = entrance (color sensor), index MAX_CAPACITY-1 = exit
 * (kicker).
 * <p>
 * Three CR wheels work in tandem:
 * - Main transfer wheel: moves balls through the slots
 * - Entrance wheel: spins to let balls in at the color sensor, holds closed
 * otherwise
 * - Exit wheel: spins to fire balls out, holds closed otherwise
 * <p>
 * Detection places a ball into the first available entrance slot and opens the
 * entrance
 * wheel briefly to allow entry. Advances shift slots toward the exit. Detection
 * uses a
 * blind window to avoid duplicate reads.
 */
public final class SingleWheelTransfer extends Mechanism {
	// Positional model: slots[0] is entrance, slots[n-1] is at exit
	private final MatchSettings.ArtifactColor[] slots = new MatchSettings.ArtifactColor[MAX_CAPACITY];
	
	// Hardware
	private final ColorSensor colorSensor;
	private final CRServo transferWheel; // Main wheel that moves balls through transfer
	private final CRServo entranceWheel; // CR wheel at color sensor that lets balls in
	private final CRServo exitWheel; // CR wheel at kicker that fires balls out
	private final FlywheelIntake intake;
	
	// Detection gating
	private long lastDetectTimeMs = 0;
	
	// Transfer wheel timing and scheduled shifts
	private boolean transferWheelRunning = false;
	private long transferWheelEndTimeMs = 0;
	private int pendingShifts = 0; // number of single-slot shifts to perform when wheel run ends
	
	// Entrance wheel timing
	private boolean entranceWheelOpen = false;
	private long entranceOpenStartTimeMs = 0;
	
	// Exit wheel timing
	private boolean exitWheelFiring = false;
	private long exitFireStartTimeMs = 0;
	
	public SingleWheelTransfer(CRServo transferWheel, CRServo entranceWheel, CRServo exitWheel,
	                           RevColorSensorV3 colorSensor, FlywheelIntake intake) {
		this.colorSensor = new ColorSensor(colorSensor);
		this.transferWheel = transferWheel;
		this.entranceWheel = entranceWheel;
		this.exitWheel = exitWheel;
		this.intake = intake;
		Arrays.fill(slots, MatchSettings.ArtifactColor.UNKNOWN);
	}
	
	@Override
	public void init() {
		holdEntranceClosed();
		holdExitClosed();
		stopTransferWheelImmediate();
	}
	
	@Override
	public void update() {
		long now = System.currentTimeMillis();
		
		// Auto-close entrance wheel after open duration
		if (entranceWheelOpen && now - entranceOpenStartTimeMs > ENTRANCE_OPEN_DURATION_MS) {
			holdEntranceClosed();
		}
		
		// Do not allow intake when we have 3 balls (violation)
		if (isFull()) {
			holdEntranceClosed();
		}
		
		// Auto-close exit wheel after fire duration
		if (exitWheelFiring && now - exitFireStartTimeMs > EXIT_FIRE_DURATION_MS) {
			holdExitClosed();
		}
		
		// Handle transfer wheel run completion and perform scheduled shifts
		if (transferWheelRunning && now >= transferWheelEndTimeMs) {
			// Stop the transfer wheel first
			stopTransferWheelImmediate();
			
			// Perform the scheduled shifts. Each shift moves every ball one index towards
			// exit.
			for (int s = 0; s < pendingShifts; s++) {
				performSingleShift();
			}
			// Handle reverse shifts (negative pendingShifts)
			for (int s = 0; s > pendingShifts; s--) {
				performSingleReverseShift();
			}
			pendingShifts = 0;
		}
		
		// Detection with blind window
		MatchSettings.ArtifactColor detected = colorSensor.getArtifactColor();
		if (detected != MatchSettings.ArtifactColor.UNKNOWN && now - lastDetectTimeMs > BLIND_WINDOW_MS) {
			lastDetectTimeMs = now;
			onBallDetected(detected);
		}
	}
	
	@Override
	public void stop() {
		stopTransferWheelImmediate();
		holdEntranceClosed();
		holdExitClosed();
		clearAllSlots();
	}
	
	/*
	 * ------------------------------
	 * Detection and positional model
	 * ------------------------------
	 */
	
	/**
	 * Called when the sensor reports a new ball (after blind window).
	 * Attempts to place the ball into the first available entrance-side slot (index
	 * 0..).
	 * If the transfer is full, the detection is ignored.
	 * Opens the entrance wheel briefly to let the ball in.
	 */
	private void onBallDetected(MatchSettings.ArtifactColor color) {
		// Place into the earliest UNKNOWN slot starting from entrance (0).
		for (int i = 0; i < slots.length; i++) {
			if (slots[i] == MatchSettings.ArtifactColor.UNKNOWN) {
				slots[i] = color;
				// Open entrance wheel to let ball in
				openEntrance();
				return;
			}
		}
		// If no slot free, ignore detection (full).
	}
	
	/**
	 * Perform a single physical shift: every slot moves one index closer to exit.
	 * Slot at exit index is not shifted beyond; it remains and must be fired to
	 * free it.
	 * After shifting, entrance slot (0) becomes UNKNOWN.
	 */
	private void performSingleShift() {
		for (int i = slots.length - 1; i >= 1; i--) {
			slots[i] = slots[i - 1];
		}
		slots[0] = MatchSettings.ArtifactColor.UNKNOWN;
	}
	
	/**
	 * Perform a single reverse shift: every slot moves one index away from exit.
	 * Move every ball one index away from exit. Entrance receives UNKNOWN.
	 */
	private void performSingleReverseShift() {
		for (int i = 0; i < slots.length - 1; i++) {
			slots[i] = slots[i + 1];
		}
		slots[slots.length - 1] = MatchSettings.ArtifactColor.UNKNOWN;
	}
	
	/*
	 * ------------------------------
	 * Transfer wheel control and advancing
	 * ------------------------------
	 */
	
	/**
	 * Request a single-step advance (move everything one position toward exit).
	 * If the wheel is currently running this will instead increment pendingShifts
	 * and extend the run time accordingly (non-destructive).
	 */
	public void advance() {
		advanceSteps(1);
	}
	
	/**
	 * Request N single-slot shifts. The transfer wheel will run for N *
	 * TRANSFER_TIME_MS total.
	 * If wheel is already running we increase pendingShifts and extend the end
	 * time.
	 */
	public void advanceSteps(int steps) {
		if (steps <= 0)
			return;
		long now = System.currentTimeMillis();
		long duration = steps * TRANSFER_TIME_MS;
		
		// If wheel already running, just accumulate shifts and extend end time.
		if (transferWheelRunning) {
			pendingShifts += steps;
			// Ensure end time is at least now + remaining duration
			transferWheelEndTimeMs = Math.max(transferWheelEndTimeMs, now + duration);
			return;
		}
		
		// Start transfer wheel forward for duration
		transferWheel.setPower(TRANSFER_WHEEL_FORWARD_POWER);
		transferWheelRunning = true;
		pendingShifts += steps;
		transferWheelEndTimeMs = now + duration;
	}
	
	/**
	 * Reverse transfer wheel to eject. This schedules the same number of shifts but
	 * in
	 * reverse direction: we move items away from exit.
	 * Implementation runs wheel in reverse for steps * TRANSFER_TIME_MS and
	 * performs
	 * reverse-shifts on completion.
	 */
	public void reverseSteps(int steps) {
		if (steps <= 0)
			return;
		long now = System.currentTimeMillis();
		long duration = steps * TRANSFER_TIME_MS;
		
		// If already running forward, ignore reverse request.
		if (transferWheelRunning)
			return;
		
		// Run wheel in reverse and mark with negative pendingShifts
		transferWheel.setPower(TRANSFER_WHEEL_REVERSE_POWER);
		transferWheelRunning = true;
		// Use negative pendingShifts to indicate reverse on completion
		pendingShifts -= steps;
		transferWheelEndTimeMs = now + duration;
	}
	
	/**
	 * Immediately stop the transfer wheel and reset running flag.
	 */
	private void stopTransferWheelImmediate() {
		transferWheel.setPower(0);
		transferWheelRunning = false;
	}
	
	/*
	 * ------------------------------
	 * Entrance wheel control
	 * ------------------------------
	 */
	
	/**
	 * Open the entrance wheel to let balls in.
	 * The wheel will automatically close after ENTRANCE_OPEN_DURATION_MS.
	 */
	private void openEntrance() {
		entranceWheel.setPower(ENTRANCE_WHEEL_INTAKE_POWER);
		intake.in();
		entranceWheelOpen = true;
		entranceOpenStartTimeMs = System.currentTimeMillis();
	}
	
	/**
	 * Hold entrance closed with slight reverse power.
	 */
	private void holdEntranceClosed() {
		entranceWheel.setPower(ENTRANCE_WHEEL_HOLD_POWER);
		intake.out();
		entranceWheelOpen = false;
	}
	
	/*
	 * ------------------------------
	 * Exit wheel control / firing
	 * ------------------------------
	 */
	
	/**
	 * Fire the ball currently at the exit index (if present).
	 * Spins the exit wheel to kick the ball out and removes it from the model
	 * immediately.
	 * Auto-closes after EXIT_FIRE_DURATION_MS.
	 */
	public void fire() {
		int exitIndex = MAX_CAPACITY - 1;
		if (exitWheelFiring)
			return; // already firing
		if (slots[exitIndex] == MatchSettings.ArtifactColor.UNKNOWN)
			return; // nothing to fire
		
		// Remove from model immediately so subsequent logic sees empty exit slot
		slots[exitIndex] = MatchSettings.ArtifactColor.UNKNOWN;
		
		// Spin exit wheel to kick ball out
		exitWheel.setPower(EXIT_WHEEL_FIRE_POWER);
		exitWheelFiring = true;
		exitFireStartTimeMs = System.currentTimeMillis();
		
		// Spin main wheel as well
		advance();
	}
	
	/**
	 * Hold exit closed with slight reverse power.
	 */
	private void holdExitClosed() {
		exitWheel.setPower(EXIT_WHEEL_HOLD_POWER);
		exitWheelFiring = false;
	}
	
	/*
	 * ------------------------------
	 * Query helpers
	 * ------------------------------
	 */
	
	/**
	 * Returns the artifact currently at the exit (next to be fired).
	 */
	public MatchSettings.ArtifactColor getNextBallToKicker() {
		return slots[MAX_CAPACITY - 1];
	}
	
	/**
	 * Returns the index of the first slot containing the provided color or -1 if
	 * not present.
	 */
	public int indexOf(MatchSettings.ArtifactColor color) {
		for (int i = 0; i < slots.length; i++) {
			if (slots[i] == color)
				return i;
		}
		return -1;
	}
	
	/**
	 * Move the specific ball (by slot index) to the exit using dynamic steps.
	 * If index is invalid or slot is UNKNOWN no action is taken.
	 */
	public void moveSlotToKicker(int slotIndex) {
		if (slotIndex < 0 || slotIndex >= slots.length)
			return;
		if (slots[slotIndex] == MatchSettings.ArtifactColor.UNKNOWN)
			return;
		int shiftsNeeded = (MAX_CAPACITY - 1) - slotIndex;
		if (shiftsNeeded > 0) {
			advanceSteps(shiftsNeeded);
		}
	}
	
	/**
	 * Returns a copy of current slots (positional snapshot).
	 */
	public MatchSettings.ArtifactColor[] getSlotsSnapshot() {
		return Arrays.copyOf(slots, slots.length);
	}
	
	public boolean isFull() {
		for (MatchSettings.ArtifactColor c : slots) {
			if (c == MatchSettings.ArtifactColor.UNKNOWN)
				return false;
		}
		return true;
	}
	
	public boolean isEmpty() {
		for (MatchSettings.ArtifactColor c : slots) {
			if (c != MatchSettings.ArtifactColor.UNKNOWN)
				return false;
		}
		return true;
	}
	
	public boolean isTransferWheelRunning() {
		return transferWheelRunning;
	}
	
	public boolean isEntranceWheelOpen() {
		return entranceWheelOpen;
	}
	
	public boolean isExitWheelFiring() {
		return exitWheelFiring;
	}
	
	/*
	 * ------------------------------
	 * Maintenance helpers
	 * ------------------------------
	 */
	
	/**
	 * Clear all slots and reset detection state.
	 */
	public void clearAllSlots() {
		Arrays.fill(slots, MatchSettings.ArtifactColor.UNKNOWN);
		lastDetectTimeMs = 0;
		pendingShifts = 0;
	}
	
	/**
	 * Convenience: compute how many shifts a ball currently at index needs to reach
	 * exit.
	 */
	public int shiftsToExitForIndex(int index) {
		if (index < 0 || index >= slots.length)
			return -1;
		return (MAX_CAPACITY - 1) - index;
	}
}
