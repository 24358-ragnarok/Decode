package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.BLIND_WINDOW_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.KICKER_LOCK_POS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.KICKER_UNLOCK_POS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.KICK_DURATION_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.MAX_CAPACITY;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.WHEEL_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.WHEEL_OUTTAKE_POWER;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

import java.util.Arrays;

/**
 * Transfer subsystem with positional awareness and detection gating.
 * <p>
 * Positions: index 0 = intake/entrance, index MAX_CAPACITY-1 = kicker/exit.
 * Detection places a ball into the first available entrance slot (ideally slot
 * 0).
 * Advances shift slots toward the kicker. Advances can be scheduled for a
 * dynamic
 * number of steps. Detection uses a blind window to avoid duplicate reads.
 */
public final class SingleWheelTransfer extends Mechanism {
	// Positional model: slots[0] is entrance, slots[n-1] is at kicker
	private final MatchSettings.ArtifactColor[] slots = new MatchSettings.ArtifactColor[MAX_CAPACITY];
	// Hardware
	private final ColorSensor colorSensor;
	private final CRServo wheelServo;
	private final Servo kickerServo;
	// Detection gating
	private long lastDetectTimeMs = 0;
	
	// Wheel timing and scheduled shifts
	private boolean wheelRunning = false;
	private long wheelRunEndTimeMs = 0;
	private int pendingShifts = 0; // number of single-slot shifts to perform when wheel run ends
	
	// Kicker timing
	private boolean kickerOpen = false;
	private long lastKickTimeMs = 0;
	
	public SingleWheelTransfer(CRServo wheel, Servo kicker, RevColorSensorV3 cs) {
		this.colorSensor = new ColorSensor(cs);
		this.wheelServo = wheel;
		this.kickerServo = kicker;
		Arrays.fill(slots, MatchSettings.ArtifactColor.UNKNOWN);
	}
	
	@Override
	public void init() {
		lockKicker();
		stopWheelImmediate();
	}
	
	@Override
	public void update() {
		long now = System.currentTimeMillis();
		
		// Auto relock kicker after kick duration
		if (kickerOpen && now - lastKickTimeMs > KICK_DURATION_MS) {
			lockKicker();
		}
		
		// Handle wheel run completion and perform scheduled shifts
		if (wheelRunning && now >= wheelRunEndTimeMs) {
			// stop the wheel first
			stopWheelImmediate();
			
			// Perform the scheduled shifts. Each shift moves every ball one index towards
			// the kicker.
			for (int s = 0; s < pendingShifts; s++) {
				performSingleShift();
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
		stopWheelImmediate();
		lockKicker();
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
	 * If the transfer is full the detection is ignored.
	 */
	private void onBallDetected(MatchSettings.ArtifactColor color) {
		// Place into the earliest UNKNOWN slot starting from entrance (0).
		for (int i = 0; i < slots.length; i++) {
			if (slots[i] == MatchSettings.ArtifactColor.UNKNOWN) {
				slots[i] = color;
				return;
			}
		}
		// If no slot free, ignore detection (full).
	}
	
	/**
	 * Perform a single physical shift: every slot moves one index closer to kicker.
	 * Slot at kicker index is not shifted beyond; it remains and must be fired to
	 * free it.
	 * After shifting, entrance slot (0) becomes UNKNOWN.
	 */
	private void performSingleShift() {
		for (int i = slots.length - 1; i >= 1; i--) {
			slots[i] = slots[i - 1];
		}
		slots[0] = MatchSettings.ArtifactColor.UNKNOWN;
	}
	
	/*
	 * ------------------------------
	 * Wheel control and advancing
	 * ------------------------------
	 */
	
	/**
	 * Request a single-step advance (move everything one position toward kicker).
	 * If the wheel is currently running this will instead increment pendingShifts
	 * and
	 * extend the run time accordingly (non-destructive).
	 */
	public void advance() {
		advanceSteps(1);
	}
	
	/**
	 * Request N single-slot shifts. The wheel will run for N * TRANSFER_TIME_MS
	 * total.
	 * If wheel is already running we increase pendingShifts and extend the end
	 * time.
	 */
	public void advanceSteps(int steps) {
		if (steps <= 0)
			return;
		long now = System.currentTimeMillis();
		long duration = steps * TRANSFER_TIME_MS;
		
		// If wheel already running, just accumulate shifts and extend end time.
		if (wheelRunning) {
			pendingShifts += steps;
			// ensure end time is at least now + remaining duration
			wheelRunEndTimeMs = Math.max(wheelRunEndTimeMs, now + duration);
			return;
		}
		
		// Start wheel forward for duration
		wheelServo.setPower(WHEEL_INTAKE_POWER);
		wheelRunning = true;
		pendingShifts += steps;
		wheelRunEndTimeMs = now + duration;
	}
	
	/**
	 * Reverse wheel to eject. This schedules the same number of shifts but in
	 * reverse direction:
	 * we move items away from kicker. Implementation here simply runs wheel in
	 * reverse for
	 * steps * TRANSFER_TIME_MS and performs reverse-shifts on completion.
	 */
	public void reverseSteps(int steps) {
		if (steps <= 0)
			return;
		long now = System.currentTimeMillis();
		long duration = steps * TRANSFER_TIME_MS;
		
		// If already running forward, ignore reverse request.
		if (wheelRunning)
			return;
		
		// Run wheel in reverse and mark wheelRunEndTimeMs with a negative pendingShifts
		wheelServo.setPower(WHEEL_OUTTAKE_POWER);
		wheelRunning = true;
		// Use negative pendingShifts to indicate reverse on completion
		pendingShifts -= steps;
		wheelRunEndTimeMs = now + duration;
	}
	
	/**
	 * Perform the reverse shifts (called when reverse run completes).
	 * Move every ball one index away from kicker. Entrance receives UNKNOWN.
	 */
	private void performSingleReverseShift() {
		for (int i = 0; i < slots.length - 1; i++) {
			slots[i] = slots[i + 1];
		}
		slots[slots.length - 1] = MatchSettings.ArtifactColor.UNKNOWN;
	}
	
	/**
	 * Immediately stop the wheel and reset running flag.
	 */
	private void stopWheelImmediate() {
		wheelServo.setPower(0);
		wheelRunning = false;
	}
	
	/*
	 * ------------------------------
	 * Kicker / firing
	 * ------------------------------
	 */
	
	/**
	 * Fire the ball currently at the kicker index (if present).
	 * Opens kicker and removes the ball immediately from the model.
	 * Auto-relocsk handled in update().
	 */
	public void fire() {
		int kickerIndex = MAX_CAPACITY - 1;
		if (kickerOpen)
			return; // already firing
		if (slots[kickerIndex] == MatchSettings.ArtifactColor.UNKNOWN)
			return; // nothing to fire
		
		// Remove from model immediately so subsequent logic sees empty kicker slot
		slots[kickerIndex] = MatchSettings.ArtifactColor.UNKNOWN;
		
		// Actuate kicker
		unlockKicker();
		kickerOpen = true;
		lastKickTimeMs = System.currentTimeMillis();
	}
	
	/*
	 * ------------------------------
	 * Query helpers
	 * ------------------------------
	 */
	
	/**
	 * Returns the artifact currently at the kicker (next to be fired).
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
	 * Move the specific ball (by slot index) to the kicker using dynamic steps.
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
	
	public boolean isWheelRunning() {
		return wheelRunning;
	}
	
	public boolean isKickerOpen() {
		return kickerOpen;
	}
	
	/*
	 * ------------------------------
	 * Kicker low-level
	 * ------------------------------
	 */
	
	private void unlockKicker() {
		kickerServo.setPosition(KICKER_UNLOCK_POS);
	}
	
	private void lockKicker() {
		kickerServo.setPosition(KICKER_LOCK_POS);
		kickerOpen = false;
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
	 * kicker.
	 */
	public int shiftsToKickerForIndex(int index) {
		if (index < 0 || index >= slots.length)
			return -1;
		return (MAX_CAPACITY - 1) - index;
	}
}
