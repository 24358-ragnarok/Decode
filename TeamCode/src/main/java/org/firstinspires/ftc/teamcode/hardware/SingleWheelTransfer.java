package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.AUTO_ADVANCE_ENABLED;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.AUTO_ADVANCE_GRACE_PERIOD_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_FIRE_DURATION_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_FIRE_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.EXIT_HOLD_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.MAX_CAPACITY;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_TIME_MS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_WHEEL_FORWARD_POWER;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Transfer.TRANSFER_WHEEL_REVERSE_POWER;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

import java.util.Arrays;

/**
 * Transfer subsystem with positional awareness and CR wheel management.
 * <p>
 * Positions: index 0 = entrance, index MAX_CAPACITY-1 = exit (kicker).
 * <p>
 * Two CR wheels work in tandem:
 * - Main transfer wheel: moves balls through the slots
 * - Exit wheel: spins to fire balls out, holds closed otherwise
 * <p>
 * Ball detection occurs at the intake location (via FlywheelIntake's color
 * sensor).
 * When a ball is detected, it is registered with a travel time delay to account
 * for
 * physical movement from intake to transfer entrance. The ball is placed into
 * the
 * first available entrance slot (slot 0). If slot 0 is occupied, balls are
 * shifted
 * forward first (if possible) before placing the new ball.
 * <p>
 * Automatic advance feature: The system automatically moves balls forward to
 * keep one ready at the exit position when possible. This happens without
 * interfering with intake operations - if a new ball is detected, it will be
 * pulled in
 * before continuing the advance. A grace period after detection ensures new
 * balls have
 * time to enter the system.
 */
public final class SingleWheelTransfer extends Mechanism {
	// Positional model: slots[0] is entrance, slots[n-1] is at exit
	public final MatchSettings.ArtifactColor[] slots = new MatchSettings.ArtifactColor[MAX_CAPACITY];

	// Hardware
	private final CRServo transferWheel; // Main wheel that moves balls through transfer
	private final CRServo exitWheel; // CR wheel at kicker that fires balls out
	private final FlywheelIntake intake;
	// Transfer wheel timing and scheduled shifts
	private boolean transferWheelRunning = false;
	private long transferWheelEndTimeMs = 0;
	private int pendingShifts = 0; // number of single-slot shifts to perform when wheel run ends
	// Exit wheel timing
	private boolean exitWheelFiring = false;
	private long exitFireStartTimeMs = 0;
	// Pending ball to place after shift completes
	private MatchSettings.ArtifactColor pendingBallAfterShift = MatchSettings.ArtifactColor.UNKNOWN;
	// Delayed ball registration from intake (travel time compensation)
	private MatchSettings.ArtifactColor pendingBallFromIntake = MatchSettings.ArtifactColor.UNKNOWN;
	private long pendingBallRegistrationTimeMs = 0;
	// Last detection time for grace period tracking
	private long lastDetectTimeMs = 0;
	
	public SingleWheelTransfer(CRServo transferWheel, CRServo exitWheel, FlywheelIntake intake) {
		this.transferWheel = transferWheel;
		this.exitWheel = exitWheel;
		this.intake = intake;
		Arrays.fill(slots, MatchSettings.ArtifactColor.UNKNOWN);

		// Set transfer reference in intake for callbacks
		if (intake != null) {
			intake.setTransfer(this);
		}
	}

	@Override
	public void start() {
		holdExitClosed();
		stopTransferWheelImmediate();
	}

	@Override
	public void update() {
		long now = System.currentTimeMillis();

		// Auto-close exit wheel after fire duration
		if (exitWheelFiring && now - exitFireStartTimeMs > EXIT_FIRE_DURATION_MS) {
			holdExitClosed();
		}

		// Handle delayed ball registration from intake (travel time compensation)
		if (pendingBallFromIntake != MatchSettings.ArtifactColor.UNKNOWN && now >= pendingBallRegistrationTimeMs) {
			onBallDetected(pendingBallFromIntake);
			pendingBallFromIntake = MatchSettings.ArtifactColor.UNKNOWN;
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

			// If there's a pending ball to place after shift, place it now
			if (pendingBallAfterShift != MatchSettings.ArtifactColor.UNKNOWN) {
				// Safety check: ensure slot 0 is empty before placing the pending ball
				// (should always be true after shift, but check for robustness)
				if (slots[0] == MatchSettings.ArtifactColor.UNKNOWN) {
					slots[0] = pendingBallAfterShift;
					pendingBallAfterShift = MatchSettings.ArtifactColor.UNKNOWN;
				} else {
					// Slot 0 is unexpectedly occupied - clear pending ball to avoid issues
					// This shouldn't happen in normal operation
					pendingBallAfterShift = MatchSettings.ArtifactColor.UNKNOWN;
				}
			}
		}

		// Automatic advance logic - keep a ball ready at exit if possible
		if (AUTO_ADVANCE_ENABLED) {
			checkAndPerformAutoAdvance(now);
		}
	}

	@Override
	public void stop() {
		stopTransferWheelImmediate();
		holdExitClosed();
		clearAllSlots();
	}

	/*
	 * ------------------------------
	 * Detection and positional model
	 * ------------------------------
	 */

	/**
	 * Called by FlywheelIntake when a ball is detected at the intake location.
	 * Schedules the ball to be registered after travel time delay.
	 *
	 * @param color              The detected artifact color
	 * @param registrationTimeMs The system time when the ball should be registered
	 *                           (now + travel time)
	 */
	public void registerBallFromIntake(MatchSettings.ArtifactColor color, long registrationTimeMs) {
		// Update last detection time for grace period tracking
		lastDetectTimeMs = System.currentTimeMillis();

		// If there's already a pending ball, replace it with the newer detection
		// (newer detection is more recent and likely more accurate)
		// This handles rapid successive detections better than ignoring them
		pendingBallFromIntake = color;
		pendingBallRegistrationTimeMs = registrationTimeMs;
	}

	/**
	 * Called when a ball should be registered in the transfer (after travel time
	 * delay).
	 * Always attempts to place the ball into slot 0 (lowest position).
	 * If slot 0 is empty, places the ball directly.
	 * If slot 0 is occupied, shifts all balls forward first (if possible), then
	 * places
	 * the new ball in slot 0 after the shift completes.
	 * If slot 0 is occupied and we cannot shift forward (exit slot is full),
	 * the detection is ignored (transfer is full).
	 */
	private void onBallDetected(MatchSettings.ArtifactColor color) {
		// If slot 0 is empty, place the ball directly without shifting
		if (slots[0] == MatchSettings.ArtifactColor.UNKNOWN) {
			slots[0] = color;
			// Start intake to help pull the ball in, but only if it's stopped
			// (don't override user's outtake command)
			if (intake != null && intake.state == FlywheelIntake.IntakeState.STOPPED) {
				intake.in();
			}
			return;
		}

		// Slot 0 is occupied. Check if we can shift forward.
		// We can shift forward only if the exit slot is not occupied.
		if (isFull()) {
			// Exit slot is full, cannot shift forward, cannot take in new ball
			// Transfer is full - ignore this detection
			return;
		}

		// If there's already a pending ball waiting for a shift, we can't queue another
		// one.
		// This should be rare, but we need to handle it.
		if (pendingBallAfterShift != MatchSettings.ArtifactColor.UNKNOWN) {
			// Already have a pending ball, cannot queue another
			return; // Will be handled after current shift completes
		}
		
		// We can shift forward. Start intake to help pull the ball in.
		// Store the new ball to be placed after the shift completes.
		// The physical wheel will move first, then we'll place the ball in slot 0.
		// Only start intake if it's stopped (don't override user's outtake command)
		if (intake != null && intake.state == FlywheelIntake.IntakeState.STOPPED) {
			intake.in();
		}
		pendingBallAfterShift = color;
		// Initiate the shift to make room for the new ball
		advance();
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
	
	public boolean canFire() {
		// simply say if a ball is in firing position and nothing is pending
		return slots[2] != MatchSettings.ArtifactColor.UNKNOWN && !transferWheelRunning && (pendingShifts == 0);
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
	
	public void reverse() {
		reverseSteps(1);
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
	 * Check conditions and automatically advance balls to keep one ready at exit.
	 * This ensures optimal firing readiness while respecting intake operations.
	 */
	private void checkAndPerformAutoAdvance(long now) {
		// Don't auto-advance if:
		// - Transfer wheel is already running
		// - Exit wheel is firing
		// - Slots not able to advance
		// - Not enough time since last detection (grace period)
		// - There's a pending ball from intake (ball is still traveling)
		if (transferWheelRunning || exitWheelFiring || isEmpty() ||
				pendingBallFromIntake != MatchSettings.ArtifactColor.UNKNOWN) {
			return;
		}
		
		// Check if we need a grace period after detection
		// This handles both transfer-started and user-started intake scenarios
		// where a ball was recently detected
		if (now - lastDetectTimeMs < AUTO_ADVANCE_GRACE_PERIOD_MS) {
			return;
		}
		
		// Check if exit slot is empty and intake slot is full
		if (slots[MAX_CAPACITY - 1] != MatchSettings.ArtifactColor.UNKNOWN
				|| slots[0] == MatchSettings.ArtifactColor.UNKNOWN) {
			return; // Exit already has a ball or entrance is empty
		}
		
		advance();
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
	 * Intake coordination (entrance wheel removed)
	 * ------------------------------
	 */
	
	/**
	 * Start intake to pull balls in. This is a convenience method that coordinates
	 * with the intake mechanism. The entrance wheel has been removed, so this
	 * only controls the intake motor.
	 */
	public void startIntake() {
		if (intake != null) {
			intake.in();
		}
	}
	
	/**
	 * Clear/outtake from entrance. Reverses intake to push balls out.
	 */
	public void clearEntrance() {
		if (intake != null) {
			intake.out();
		}
		lastDetectTimeMs = System.currentTimeMillis();
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
	 * Auto-stops after EXIT_FIRE_DURATION_MS.
	 */
	public void fire() {
		int exitIndex = MAX_CAPACITY - 1;
		if (exitWheelFiring)
			return; // already firing
		
		// Remove from model immediately so subsequent logic sees empty exit slot
		slots[exitIndex] = MatchSettings.ArtifactColor.UNKNOWN;
		
		// Spin exit wheel to kick ball out
		exitWheel.setPower(EXIT_FIRE_POWER);
		exitWheelFiring = true;
		exitFireStartTimeMs = System.currentTimeMillis();
	}
	
	/**
	 * Hold exit closed (stop the wheel or apply slight reverse power if needed).
	 */
	private void holdExitClosed() {
		exitWheel.setPower(EXIT_HOLD_POWER);
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
		if (shiftsNeeded - pendingShifts > 0) {
			advanceSteps(shiftsNeeded - pendingShifts);
		}
	}
	
	/**
	 * Rotate the transfer wheel to bring the closest ball (closest to launch)
	 * to the launch position (exit). If no ball is present, no action is taken.
	 */
	public void moveNextBallToKicker() {
		// Find the closest ball from exit (largest index with a ball)
		for (int i = slots.length - 1; i >= 0; i--) {
			if (slots[i] != MatchSettings.ArtifactColor.UNKNOWN) {
				moveSlotToKicker(i);
				return;
			}
		}
		// No ball found, nothing to do
	}
	
	/**
	 * Returns a copy of current slots (positional snapshot).
	 */
	public MatchSettings.ArtifactColor[] getSlotsSnapshot() {
		return Arrays.copyOf(slots, slots.length);
	}
	
	public boolean isFull() {
		// We're full if we cannot take in a new ball.
		// Since shifting always moves all balls forward, we can only shift forward
		// (and thus make room for a new ball in slot 0) when the exit slot is empty.
		// If the exit slot is occupied, we cannot shift forward (would push a ball
		// beyond the exit), so we're full.
		int exitIndex = MAX_CAPACITY - 1;
		return slots[exitIndex] != MatchSettings.ArtifactColor.UNKNOWN;
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
		pendingBallAfterShift = MatchSettings.ArtifactColor.UNKNOWN;
		pendingBallFromIntake = MatchSettings.ArtifactColor.UNKNOWN;
		pendingBallRegistrationTimeMs = 0;
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
