package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.Settings;

@SuppressWarnings({"ClassHasNoToStringMethod", "ClassWithoutNoArgConstructor"})
public class Lever extends Mechanism {
	private final ServoImplEx leftExtender;
	private final ServoImplEx rightExtender;
	private ParkState state;
	
	public Lever(ServoImplEx l, ServoImplEx r) {
		this.leftExtender = l;
		this.rightExtender = r;
		this.state = ParkState.RETRACTED;
	}
	
	public final void start() {
		retract();
	}
	
	public final void update() {
	}
	
	@Override
	public void stop() {
	}
	
	public final void extend() {
		leftExtender.setPosition(Settings.Lever.LEFT_SERVO_DEPLOY_POS);
		rightExtender.setPosition(Settings.Lever.RIGHT_SERVO_DEPLOY_POS);
		state = ParkState.EXTENDED;
	}
	
	public final void retract() {
		leftExtender.setPosition(Settings.Lever.LEFT_SERVO_RETRACTED_POS);
		rightExtender.setPosition(Settings.Lever.RIGHT_SERVO_RETRACTED_POS);
		state = ParkState.RETRACTED;
	}
	
	public boolean isExtended() {
		return state == ParkState.EXTENDED;
	}
	
	enum ParkState {
		RETRACTED,
		EXTENDED
	}
}