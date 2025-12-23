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
	 * Default runtime: Launch preloads, collect 2 sets of balls, launch them, then
	 * park.
	 * This is the "safe" competition runtime.
	 */
	BEST("BEST (9ball)") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					.scanObelisk()
					// Launch preloads
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.launch()
					
					// Get ball set I
					.moveSplineTo(Settings.Positions.Samples.Preset1.PREP,
							"Prep Preset 1",
							Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR)
					.startPickup()
					.moveSlowlyTo(Settings.Positions.Samples.Preset1.GRAB_1, "Grab Preset 1 Ball 1")
					.moveSlowlyTo(Settings.Positions.Samples.Preset1.GRAB_2, "Grab Preset 1 Ball 2")
					.moveSlowlyTo(Settings.Positions.Samples.Preset1.END, "Grab Preset 1 Ball 3")
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
					.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_1, "Grab1 Preset2")
					
					.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_2, "Grab2 Preset2")
					
					.moveSlowlyTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
					
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
					.moveTo(Settings.Positions.Towers.SCAN)
					.scanObelisk()
					// Launch preload
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT, "Launch Preload")
					
					.launch()
					
					// Get ball set I (Preset3 for close sequence)
					.moveTo(Settings.Positions.Samples.Preset3.PREP, "Prep Preset3")
					.startPickup()
					.moveSlowlyTo(Settings.Positions.Samples.Preset3.GRAB_1, "Grab1 Preset3")
					
					.moveSlowlyTo(Settings.Positions.Samples.Preset3.GRAB_2, "Grab2 Preset3")
					
					.moveSlowlyTo(Settings.Positions.Samples.Preset3.END, "End Preset3")
					
					// .endPickup()
					
					// Launch ball set I
					.prepLaunch()
					.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
							Settings.Positions.ControlPoints.FROM_PRESET3_TO_CLOSE, "Launch Preset3")
					
					.launch()
					
					// Get ball set II (Preset2 for close sequence)
					.moveTo(Settings.Positions.Samples.Preset2.PREP, "Prep Preset2")
					.startPickup()
					.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_1, "Grab1 Preset2")
					
					.moveSlowlyTo(Settings.Positions.Samples.Preset2.GRAB_2, "Grab2 Preset2")
					
					.moveSlowlyTo(Settings.Positions.Samples.Preset2.END, "End Preset2")
					
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
	JUST_LAUNCH("Just Launch & Park") {
		@Override
		public AutonomousSequence buildFarSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.launch()
					
					.moveTo(Settings.Positions.Default.FAR_SAFE_PARK_POSE, "Park")
					.endAt(Settings.Positions.Default.FAR_SAFE_PARK_POSE)
					.build();
		}
		
		@Override
		public AutonomousSequence buildCloseSequence() {
			return new SequenceBuilder()
					.prepLaunch()
					.moveTo(Settings.Positions.TeleOp.FAR_SHOOT, "Launch Preload")
					.launch()
					
					.moveTo(Settings.Positions.Default.FAR_SAFE_PARK_POSE, "Park")
					.endAt(Settings.Positions.Default.CLOSE_SAFE_PARK_POSE)
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
	};
	
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
}
