package org.firstinspires.ftc.teamcode.software;

import com.pedropathing.util.Timer;

public class SlewThrottle {
	public static final double MAX_SLEW = 3;
	public double lastDrive;
	public double lastStrafe;
	public double lastRotate;
	public Timer delta = new Timer();
	
	public SlewThrottle() {
		lastDrive = 0;
		lastStrafe = 0;
		lastRotate = 0;
		delta.resetTimer();
	}
	
	public double[] throttled(double drive, double strafe, double rotate) {
		double realSlew = MAX_SLEW * delta.getElapsedTimeSeconds();
		
		double driveDelta = drive - lastDrive;
		double strafeDelta = strafe - lastStrafe;
		double rotateDelta = rotate - lastRotate;
		
		// Clamp change rate per call
		if (driveDelta > realSlew) driveDelta = realSlew;
		if (driveDelta < -realSlew) driveDelta = -realSlew;
		if (strafeDelta > realSlew) strafeDelta = realSlew;
		if (strafeDelta < -realSlew) strafeDelta = -realSlew;
		if (rotateDelta > realSlew) rotateDelta = realSlew;
		if (rotateDelta < -realSlew) rotateDelta = -realSlew;
		
		// Apply incremental update
		lastDrive += driveDelta;
		lastStrafe += strafeDelta;
		lastRotate += rotateDelta;
		
		delta.resetTimer();
		
		return new double[]{lastDrive, lastStrafe, lastRotate};
	}
}
