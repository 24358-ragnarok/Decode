package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Swap.GRABBING_POS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Swap.HOLDING_POS;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BallSwap extends Mechanism {
	private final ServoImplEx servo;
	private final MechanismManager mechanisms;
	private SwapState state;
	
	public BallSwap(MechanismManager mechanisms,
	                ServoImplEx swap) {
		this.mechanisms = mechanisms;
		this.servo = swap;
		this.state = SwapState.UNKNOWN;
	}
	
	public final void start() {
		hold();
	}
	
	public final void update() {
	}
	
	public final void swap() {
		if (state == SwapState.GRABBING) {
			hold();
		} else if (state == SwapState.HOLDING) {
			grab();
		}
	}
	
	public final void grab() {
		servo.setPosition(GRABBING_POS);
		state = SwapState.GRABBING;
	}
	
	public final void hold() {
		servo.setPosition(HOLDING_POS);
		state = SwapState.HOLDING;
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