package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Positions.ControlPoints.EMPTY_GATE_APPROACH;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET2_END;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Positions.Samples.GateAndEating.EMPTY_GATE;

import org.firstinspires.ftc.teamcode.configuration.Settings;

/**
 * Defines different autonomous runtime strategies.
 * Each runtime provides both a FAR and CLOSE sequence variant.
 * <p>
 * Use the MatchConfigurationWizard to select the desired runtime before the
 * match.
 */
public enum AutonomousRuntime {
	SORTED("Sorted 9 Ball") {
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
	CLASSIC("Classic 9 Ball") {
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
	BEST("[CB] Closest two presets and loop eat combined combined auto") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.KRAKATOA()
					
					// Get ball set I
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset1.END,
							Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR, "Prep Preset 1")
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT,
							Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR, "Launch Preset1")
					.KRAKATOA()
					
					
					// Loop: Get balls from HP and launch until 5 seconds left
					.loopUntilSecondsLeft(3, loop -> loop
							.moveCurveToVia(Settings.Positions.Samples.HumanPlayerPreset.PREP,
									Settings.Positions.ControlPoints.HUMAN_PLAYER,
									"Prep Human Player")
							.startPickup()
							.moveTo(Settings.Positions.Samples.HumanPlayerPreset.END,
									"End Human Player")
							.prepLaunch()
							.moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT,
									Settings.Positions.ControlPoints.HUMAN_PLAYER_TO_FAR_SHOOT, "Human player to far shoot")
							.KRAKATOA())
					
					// Park
					.endPickup()
					.moveTo(Settings.Positions.Park.FAR, "Park")
					.endAt(Settings.Positions.Park.FAR)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					.KRAKATOA()
					
					// Get ball set I (Preset2 for close sequence)
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset2.END_AND_EMPTY_GATE,
							FROM_CLOSE_SHOOT_TO_PRESET2_END, "Prep Preset2")
					
					// .endPickup()
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							FROM_CLOSE_SHOOT_TO_PRESET2_END, "Launch Preset3")
					
					.KRAKATOA()
					
					
					// Loop: Get balls from eat and launch until 6 seconds left
					.loopUntilSecondsLeft(6, loop -> loop
							.startPickup()
							.moveCurveToVia(EMPTY_GATE, EMPTY_GATE_APPROACH,
									"Curve to empty gate")
							.wait(1.2)
							.prepLaunch()
							.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
									EMPTY_GATE_APPROACH,
									"Launch Direct Eat")
							.KRAKATOA())
					
					// Get ball set II (Preset3 for close sequence)
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset3.END,
							Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET3_END, "Launch Preset3")
					
					
					// Launch ball set II
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET3_END, "Launch Preset2")
					.KRAKATOA()
					
					.endPickup()
					
					.moveTo(Settings.Positions.Park.CLOSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE)
					.build();
		}
	},
	SOLO("18 Ball Solo Auto Close only don't for the love of god don't use far") {
		@Override
		public boolean supportsFar() {
			return false; // Only Close sequence is implemented
		}
		@Override
		public AutonomousSequence buildFarSequence() {
			throw new UnsupportedOperationException("Far sequence not supported for 18 ball close runtime");
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					
					.KRAKATOA()
					
					// Get ball set I (Preset2 for close sequence)
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset2.END_AND_EMPTY_GATE,
							FROM_CLOSE_SHOOT_TO_PRESET2_END, "Prep Preset2")
					
					// .endPickup()
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							FROM_CLOSE_SHOOT_TO_PRESET2_END, "Launch Preset3")
					
					.KRAKATOA()
					
					// Get balls from eat and shoot
					.startPickup()
					.moveCurveToVia(EMPTY_GATE, EMPTY_GATE_APPROACH,
							"Curve to empty gate")
					.wait(.5)
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							EMPTY_GATE_APPROACH,
							"Launch Direct Eat")
					.KRAKATOA()
					
					// Get ball set II (Preset3 for close sequence)
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset3.END,
							Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET3_END, "Launch Preset3")
					
					
					// Launch ball set II
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET3_END, "Launch Preset2")
					.KRAKATOA()
					
					
					// Loop: Get balls from HP and launch until 5 seconds left
					.loopUntilSecondsLeft(.3, loop -> loop
							.startPickup()
							.moveCurveToVia(EMPTY_GATE, EMPTY_GATE_APPROACH,
									"Curve to empty gate")
							.wait(.5)
							.prepLaunch()
							.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
									EMPTY_GATE_APPROACH,
									"Launch Direct Eat")
							.KRAKATOA())
					
					.endPickup()
					
					.moveTo(Settings.Positions.Park.CLOSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE)
					.build();
		}
	},
	
	CONNER("conner's 15 ball secured close auto") {
		
		@Override
		public boolean supportsFar() {
			return false; // Only Close sequence is implemented
		}
		@Override
		public AutonomousSequence buildFarSequence() {
			throw new UnsupportedOperationException("Far sequence not supported for CONNER runtime");
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					
					.KRAKATOA()
					
					// Get ball set I (Preset2 for close sequence)
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset2.END_AND_EMPTY_GATE,
							FROM_CLOSE_SHOOT_TO_PRESET2_END, "Prep Preset2")
					
					// .endPickup()
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							FROM_CLOSE_SHOOT_TO_PRESET2_END, "Launch Preset3")
					
					.KRAKATOA()
					
					// Get balls from eat and shoot
					.startPickup()
					.moveCurveToVia(EMPTY_GATE, EMPTY_GATE_APPROACH,
							"Curve to empty gate")
					.wait(.5)
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							EMPTY_GATE_APPROACH,
							"Launch Direct Eat")
					.KRAKATOA()
					
					// Get ball set II (Preset3 for close sequence)
					.startPickup()
					.moveCurveToVia(Settings.Positions.Samples.Preset3.END,
							Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET3_END, "Launch Preset3")
					
					
					// Launch ball set II
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET3_END, "Launch Preset2")
					.KRAKATOA()
					
					
					// Loop: Get balls from HP and launch until 5 seconds left
					.loopUntilSecondsLeft(4.0, loop -> loop
							.startPickup()
							.moveCurveToVia(EMPTY_GATE, EMPTY_GATE_APPROACH,
									"Curve to empty gate")
							.wait(.5)
							.prepLaunch()
							.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
									EMPTY_GATE_APPROACH,
									"Launch Direct Eat")
							.KRAKATOA())
					
					.endPickup()
					
					.moveTo(Settings.Positions.Park.CLOSE, "Park")
					.endAt(Settings.Positions.Park.CLOSE)
					.build();
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
