package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Wrapper around ServoImplEx that enforces a simple time-based busy state after
 * a command.
 */
@SuppressWarnings({"ClassHasNoToStringMethod", "ClassWithoutNoArgConstructor"})
public class TimelockedServo {
	private final ServoImplEx delegate;
	private final long cooldownMs;
	private final Timer timer;
	
	public TimelockedServo(ServoImplEx delegate, long cooldownMs) {
		this.delegate = delegate;
		this.cooldownMs = cooldownMs;
		this.timer = new Timer();
		this.timer.resetTimer();
	}
	
	public double getPosition() {
		return delegate.getPosition();
	}
	
	public void setPosition(double position) {
		delegate.setPosition(position);
		timer.resetTimer();
	}
	
	public boolean isBusy() {
		return timer.getElapsedTime() < cooldownMs;
	}
	
	@SuppressWarnings("unused")
	public ServoImplEx getDelegate() {
		return delegate;
	}
}
