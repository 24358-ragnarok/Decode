package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.configuration.MatchSettings;

public class DualBallCompartment extends Mechanism {
	private final Compartment left;
	private final Compartment right;
	private final MechanismManager mechanisms;
	
	public DualBallCompartment(MechanismManager mechanisms,
	                           ServoImplEx compartmentLeft,
	                           ServoImplEx compartmentRight) {
		this.mechanisms = mechanisms;
		this.left = new Compartment(compartmentLeft);
		this.right = new Compartment(compartmentRight);
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
	
	
	public class Compartment extends Mechanism {
		public MatchSettings.ArtifactColor stored;
		public ServoImplEx servo;
		public double activationTime;
		
		public Compartment(ServoImplEx servo) {
			this.servo = servo;
			stored = MatchSettings.ArtifactColor.UNKNOWN;
			activationTime = System.currentTimeMillis();
		}
		
		public void store(MatchSettings.ArtifactColor color) {
			servo.setPosition(1);
			stored = color;
			activationTime = System.currentTimeMillis();
		}
		
		public MatchSettings.ArtifactColor release() {
			servo.setPosition(1);
			MatchSettings.ArtifactColor released = stored;
			stored = MatchSettings.ArtifactColor.UNKNOWN;
			activationTime = System.currentTimeMillis();
			return released;
		}
		
		public MatchSettings.ArtifactColor getStoredBall() {
			return stored;
		}
		
		@Override
		public void start() {
			servo.setPosition(1);
		}
		
		@Override
		public void update() {
		
		}
		
		@Override
		public void stop() {
		}
	}
	
}