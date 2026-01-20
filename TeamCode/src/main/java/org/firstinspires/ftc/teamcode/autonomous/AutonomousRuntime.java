package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.configuration.Settings;

/**
 * Defines different autonomous runtime strategies.
 * Each runtime provides both a FAR and CLOSE sequence variant.
 * <p>
 * Use the MatchConfigurationWizard to select the desired runtime before the
 * match.
 */
public enum AutonomousRuntime {
	/**
	 * 9-ball autonomous that scans, then launches in motif order using swap.
	 */
	SORTED_NINE_BALL("Sorted 9 Ball") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					// Scan from far pose
					.prepLaunch()
					.moveTo(Settings.Positions.Towers.FAR_SCAN, "Scan (Far)")
					.scanObelisk()
					
					// Launch preloads (sorted)
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.sortedLaunch()
					
					// Get ball set I
					.moveSplineTo(Settings.Positions.Samples.Preset1.PREP,
							"Prep Preset 1",
							Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR)
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset1.END, "Grab Preset 1 Ball 3")
					
					// Launch ball set I (sorted)
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET3_TO_FAR, "Launch Preset1")
					.sortedLaunch()
					
					// Get ball set II
					.moveSplineTo(Settings.Positions.Samples.Preset2.PREP,
							"Prep Preset2",
							Settings.Positions.ControlPoints.PRESET_2_APPROACH_FAR)
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
					
					// Launch ball set II (sorted)
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preset2")
					.sortedLaunch()
					
					// Park
					.endPickup()
					.moveTo(Settings.Positions.Park.FAR, "Park")
					.endAt(Settings.Positions.Park.FAR)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					// Scan from close pose
					
					.prepLaunch()
					.moveTo(Settings.Positions.Towers.CLOSE_SCAN, "Scan (Close)")
					.scanObelisk()
					
					// Launch preload (sorted)
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					.sortedLaunch()
					
					// Get ball set I (Preset3 for close sequence)
					.moveTo(Settings.Positions.Samples.Preset3.PREP, "Prep Preset3")
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset3.END, "End Preset3")
					
					// Launch ball set I (sorted)
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET3_TO_CLOSE, "Launch Preset3")
					.sortedLaunch()
					
					// Get ball set II (Preset2 for close sequence)
					.moveTo(Settings.Positions.Samples.Preset2.PREP, "Prep Preset2")
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
					
					// Launch ball set II (sorted)
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET2_TO_CLOSE, "Launch Preset2")
					.sortedLaunch()
					
					// Park
					.endPickup()
					.moveTo(Settings.Positions.Park.CLOSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE)
					.build();
		}
	},
	
	/**
	 * Reset runtime: Launch preloads, collect 2 sets of balls, launch them, then
	 * park.
	 * This is the "safe" competition runtime.
	 */
	BEST("Classic 9 Ball") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.launch()
					
					// Get ball set I
					.moveSplineTo(Settings.Positions.Samples.Preset1.PREP,
							"Prep Preset 1",
							Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR)
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset1.END, "Grab Preset 1 Ball 3")
					// .endPickup()
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET3_TO_FAR, "Launch Preset1")
					.launch()
					
					// Get ball set II
					.moveSplineTo(Settings.Positions.Samples.Preset2.PREP,
							"Prep Preset2",
							Settings.Positions.ControlPoints.PRESET_2_APPROACH_FAR)
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
					
					// .endPickup()
					
					// Launch ball set II
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preset2")
					
					.launch()
					
					// Park
					.endPickup()
					.moveTo(Settings.Positions.Park.FAR, "Park")
					.endAt(Settings.Positions.Park.FAR)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					
					// Launch preload
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					
					.launch()
					
					// Get ball set I (Preset3 for close sequence)
					.moveTo(Settings.Positions.Samples.Preset3.PREP, "Prep Preset3")
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset3.END, "End Preset3")
					
					// .endPickup()
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET3_TO_CLOSE, "Launch Preset3")
					
					.launch()
					
					// Get ball set II (Preset2 for close sequence)
					.moveTo(Settings.Positions.Samples.Preset2.PREP, "Prep Preset2")
					.startPickup()
					
					.moveTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
					
					// .endPickup()
					
					// Launch ball set II
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET2_TO_CLOSE, "Launch Preset2")
					
					.launch()
					
					// Park
					.endPickup()
					.moveTo(Settings.Positions.Park.CLOSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE)
					.build();
		}
	},
	CIRCUIT("[CIRCUIT] preload/set1/HP loop") {
		@Override
		public boolean supportsClose() {
			return false; // Only FAR sequence is implemented
		}
		
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.KRAKATOA()
					
					// Get ball set I
					.moveSplineTo(Settings.Positions.Samples.Preset1.PREP,
							"Prep Preset 1",
							Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR)
					.startPickup()
					.moveTo(Settings.Positions.Samples.Preset1.END, "Grab Preset 1 Ball 3")
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET3_TO_FAR, "Launch Preset1")
					.KRAKATOA()
					
					
					// Loop: Get balls from HP and launch until 5 seconds left
					.loopUntilSecondsLeft(5, loop -> loop
							.moveCurveToVia(Settings.Positions.Samples.HumanPlayerPreset.PREP,
									Settings.Positions.ControlPoints.HUMAN_PLAYER,
									"Prep Human Player")
							.startPickup()
							.moveSlowlyTo(Settings.Positions.Samples.HumanPlayerPreset.END,
									"End Human Player")
							.prepLaunch()
							.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Eat Kalya's Balls")
							.KRAKATOA())
					
					// Park
					.endPickup()
					.moveTo(Settings.Positions.Park.FAR, "Park")
					.endAt(Settings.Positions.Park.FAR)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			throw new UnsupportedOperationException("CIRCUIT runtime does not support CLOSE position");
		}
	},
	
	JUST_LAUNCH("Just Launch & Park") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.launch()
					
					.moveTo(Settings.Positions.Park.FAR_SAFE_PARK_POSE, "Park")
					.endAt(Settings.Positions.Park.FAR_SAFE_PARK_POSE)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					.launch()
					
					.moveTo(Settings.Positions.Park.CLOSE_SAFE_PARK_POSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE_SAFE_PARK_POSE)
					.build();
		}
	},
	
	/**
	 * Just Park runtime: Minimal autonomous that only parks.
	 * Use this when mechanisms are unreliable or time is extremely limited.
	 */
	JUST_PARK("Just Park") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					
					.moveTo(Settings.Positions.Park.FAR, "Park")
					.endAt(Settings.Positions.Park.FAR)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					
					.moveTo(Settings.Positions.Park.CLOSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE)
					.build();
		}
	},
	;
	
	private final String displayName;
	
	AutonomousRuntime(String displayName) {
		this.displayName = displayName;
	}
	
	/**
	 * @return Human-readable name for telemetry display
	 */
	public String getDisplayName() {
		return displayName;
	}
	
	/**
	 * Builds the autonomous sequence for FAR starting position.
	 *
	 * @return The built AutonomousSequence
	 */
	public abstract AutonomousSequence buildFarSequence();
	
	/**
	 * Builds the autonomous sequence for CLOSE starting position.
	 *
	 * @return The built AutonomousSequence
	 */
	public abstract AutonomousSequence buildCloseSequence();
	
	/**
	 * @return true if this runtime supports FAR starting position
	 */
	public boolean supportsFar() {
		return true;
	}
	
	/**
	 * @return true if this runtime supports CLOSE starting position
	 */
	public boolean supportsClose() {
		return true;
	}
	
	/**
	 * Checks if this runtime supports the given starting position.
	 *
	 * @param position The starting position to check
	 * @return true if supported
	 */
	public boolean supportsPosition(
			org.firstinspires.ftc.teamcode.configuration.MatchState.AutoStartingPosition position) {
		return position == org.firstinspires.ftc.teamcode.configuration.MatchState.AutoStartingPosition.FAR
				? supportsFar()
				: supportsClose();
	}
	
	/**
	 * Gets the next runtime in the cycle (for d-pad navigation).
	 *
	 * @return The next runtime
	 */
	public AutonomousRuntime next() {
		AutonomousRuntime[] values = values();
		return values[(this.ordinal() + 1) % values.length];
	}
	
	/**
	 * Gets the previous runtime in the cycle (for d-pad navigation).
	 *
	 * @return The previous runtime
	 */
	public AutonomousRuntime previous() {
		AutonomousRuntime[] values = values();
		return values[(this.ordinal() - 1 + values.length) % values.length];
	}
	
	/**
	 * Gets the next runtime that supports the given position.
	 *
	 * @param position The starting position that must be supported
	 * @return The next compatible runtime
	 */
	public AutonomousRuntime nextFor(
			org.firstinspires.ftc.teamcode.configuration.MatchState.AutoStartingPosition position) {
		AutonomousRuntime candidate = this.next();
		int attempts = 0;
		while (!candidate.supportsPosition(position) && attempts < values().length) {
			candidate = candidate.next();
			attempts++;
		}
		return candidate;
	}
	
	/**
	 * Gets the previous runtime that supports the given position.
	 *
	 * @param position The starting position that must be supported
	 * @return The previous compatible runtime
	 */
	public AutonomousRuntime previousFor(
			org.firstinspires.ftc.teamcode.configuration.MatchState.AutoStartingPosition position) {
		AutonomousRuntime candidate = this.previous();
		int attempts = 0;
		while (!candidate.supportsPosition(position) && attempts < values().length) {
			candidate = candidate.previous();
			attempts++;
		}
		return candidate;
	}
}
