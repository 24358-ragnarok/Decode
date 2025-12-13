package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.software.game.Artifact;

public class DualBallCompartment extends Mechanism {
	private final Cartridge left;
	private final Cartridge right;
	private final MechanismManager mechanisms;
	
	public DualBallCompartment(MechanismManager mechanisms,
	                           ServoImplEx compartmentLeft,
	                           ServoImplEx compartmentRight) {
		this.mechanisms = mechanisms;
		this.left = new Cartridge(compartmentLeft);
		this.right = new Cartridge(compartmentRight);
	}
	
	public final void start() {
		left.start();
		right.start();
	}
	
	public final void update() {
		left.update();
		right.update();
	}
	
	@Override
	public void stop() {
		left.stop();
		right.stop();
	}
	
	
	public static class Cartridge extends Mechanism {
		public Artifact stored;
		public ServoImplEx servo;
		public double activationTime;
		
		public Cartridge(ServoImplEx servo) {
			this.servo = servo;
			stored = Artifact.NONE;
			activationTime = System.currentTimeMillis();
		}
		
		public void store(Artifact color) {
			servo.setPosition(1);
			stored = color;
			activationTime = System.currentTimeMillis();
		}
		
		public Artifact release() {
			servo.setPosition(1);
			Artifact released = stored;
			stored = Artifact.NONE;
			activationTime = System.currentTimeMillis();
			return released;
		}
		
		public Artifact getStoredBall() {
			return stored;
		}
		
		@Override
		public void start() {
			servo.setPosition(1); // lock by default
		}
		
		@Override
		public void update() {
		
		}
		
		@Override
		public void stop() {
		}
	}
	
}