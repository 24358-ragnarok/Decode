package org.firstinspires.ftc.teamcode.configuration;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Constants and configuration for the Pedro Pathing system.
 * <p>
 * Contains all the tuned parameters for:
 * - Follower control (PIDF coefficients, mass, acceleration)
 * - Mecanum drivetrain configuration (motor names, directions, velocities)
 * - Pinpoint localizer setup (pod positions, encoder configuration)
 * - Path constraints (velocity, acceleration, timing limits)
 * <p>
 * These values should be tuned using the Pedro Pathing tuning OpModes
 * before use in competition.
 */
public class Constants {
	public static final FollowerConstants followerConstants = new FollowerConstants()
			.mass(13.8)
			.forwardZeroPowerAcceleration(-36.00)
			.lateralZeroPowerAcceleration(-73.00)
			
			.translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0.00001, 0.02, 0.02))
			.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.02, 0.02))
			
			.headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.001, 0.05, 0.03))
			.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.65, 0.001, 0.015, 0.02))
			
			.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.65, 0.0, 0.2, 0.6, 0.0))
			.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.15, 0.0, 0.01, 0.6, 0.0))
			
			.centripetalScaling(0.0003)
			
			.automaticHoldEnd(true)
			.useSecondaryDrivePIDF(true)
			.useSecondaryHeadingPIDF(true)
			.useSecondaryTranslationalPIDF(true);
	
	public static final MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.leftFrontMotorName(Settings.Hardware.FRONT_LEFT_MOTOR.getName())
			.leftRearMotorName(Settings.Hardware.REAR_LEFT_MOTOR.getName())
			.rightFrontMotorName(Settings.Hardware.FRONT_RIGHT_MOTOR.getName())
			.rightRearMotorName(Settings.Hardware.REAR_RIGHT_MOTOR.getName())
			.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
			.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
			.xVelocity(88.0).yVelocity(61.0)
			.useVoltageCompensation(false)
			.nominalVoltage(13)
			.useBrakeModeInTeleOp(false);
	
	public static PinpointConstants localizerConstants = new PinpointConstants()
			.forwardPodY(-3.95)
			.strafePodX(3.7)
			.distanceUnit(DistanceUnit.INCH)
			.hardwareMapName(Settings.Hardware.PINPOINT.getName())
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
			.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
	
	/**
	 * Path constraints that control how the robot follows paths.
	 * <p>
	 * Parameters in order:
	 * - tValueConstraint: How close to path end before considering complete (0.998)
	 * - velocityConstraint: Minimum velocity threshold (0.01)
	 * - translationalConstraint: Translational tolerance (0.01)
	 * - headingConstraint: Heading tolerance in radians (0.001)
	 * - timeoutConstraint: Maximum time for path completion in seconds (50)
	 * - brakingStrength: How aggressively to brake at path end (2)
	 * - BEZIER_CURVE_SEARCH_LIMIT: Search resolution for curves (10 - don't change)
	 * - brakingStart: When to start braking relative to path end (1)
	 */
	public static PathConstraints pathConstraints = new PathConstraints(
			0.995,
			0.01,
			0.01,
			0.001,
			200,
			2.2,
			10,
			0.75);
	
	/**
	 * Factory method to create a configured Follower instance.
	 *
	 * @param hardwareMap The robot's hardware map
	 * @return A fully configured Follower ready for use
	 */
	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
				.mecanumDrivetrain(driveConstants)
				.pathConstraints(pathConstraints)
				.pinpointLocalizer(localizerConstants)
				.build();
	}
}