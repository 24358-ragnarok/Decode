package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.configuration.MatchSettings;
import org.firstinspires.ftc.teamcode.hardware.MechanismManager;
import org.firstinspires.ftc.teamcode.hardware.SingleWheelTransfer;

import java.util.Arrays;

/**
 * Override action that sets all transfer slots to full with a specified
 * artifact color.
 * This is useful for testing and debugging autonomous sequences without
 * requiring
 * physical ball collection.
 */
public class OverrideTransferState implements AutonomousAction {
	private final MatchSettings.ArtifactColor artifactColor;
	
	/**
	 * Creates an override action that fills all transfer slots with the specified
	 * color.
	 *
	 * @param artifactColor The color to fill all slots with
	 */
	public OverrideTransferState(MatchSettings.ArtifactColor artifactColor) {
		this.artifactColor = artifactColor;
	}
	
	/**
	 * Creates an override action that fills all transfer slots with PURPLE
	 * artifacts.
	 * This is a convenience constructor for the most common test case.
	 */
	public OverrideTransferState() {
		this(MatchSettings.ArtifactColor.PURPLE);
	}
	
	@Override
	public void initialize(MechanismManager mechanisms) {
		SingleWheelTransfer transfer = mechanisms.get(SingleWheelTransfer.class);
		if (transfer != null) {
			Arrays.fill(transfer.slots, artifactColor);
		}
	}
	
	@Override
	public boolean execute(MechanismManager mechanisms) {
		// This action completes immediately after setting the state
		return true;
	}
	
	@Override
	public void end(MechanismManager mechanisms, boolean interrupted) {
		// Nothing to clean up - the override state persists
	}
	
	@Override
	public String getName() {
		return "OverrideTransferState(" + artifactColor + ")";
	}
}
