package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Swap.GRABBING_POS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Swap.HOLDING_POS;

import org.firstinspires.ftc.teamcode.software.game.Artifact;

@SuppressWarnings({"ClassHasNoToStringMethod", "ClassWithoutNoArgConstructor"})
public class BallSwap extends Mechanism {
	private final TimelockedServo servo;
	@SuppressWarnings({"FieldCanBeLocal", "unused"})
	private final MechanismManager mechanisms;
	private SwapState state;
	private Artifact heldArtifact = Artifact.NONE;
	
	public BallSwap(MechanismManager mechanisms,
	                TimelockedServo swap) {
		this.mechanisms = mechanisms;
		this.servo = swap;
		this.state = SwapState.UNKNOWN;
	}
	
	public final void start() {
		moveToHold();
	}
	
	public final void update() {
	}
	
	@SuppressWarnings("unused")
	public final void swap() {
		if (state == SwapState.GRABBING) {
			moveToHold();
		} else if (state == SwapState.HOLDING) {
			moveToTransfer();
		}
	}
	
	public final void moveToTransfer() {
		servo.setPosition(GRABBING_POS);
		state = SwapState.GRABBING;
	}
	
	public final void moveToHold() {
		servo.setPosition(HOLDING_POS);
		state = SwapState.HOLDING;
	}
	
	@SuppressWarnings("MethodWithMultipleReturnPoints")
	public void storeArtifact(Artifact artifact) {
		if (artifact == null) {
			return;
		}
		artifact.beingSwapped = true;
		heldArtifact = artifact;
	}
	
	public Artifact takeHeldArtifact() {
		Artifact out = heldArtifact;
		heldArtifact = Artifact.NONE;
		return out;
	}
	
	public Artifact peekHeldArtifact() {
		return heldArtifact;
	}
	
	public boolean isBusy() {
		return servo.isBusy();
	}
	
	public boolean hasHeldArtifact() {
		return heldArtifact != null && heldArtifact.color != Artifact.Color.NONE;
	}
	
	@Override
	public void stop() {
	}
	
	enum SwapState {
		GRABBING,
		HOLDING,
		UNKNOWN
		
	}
}