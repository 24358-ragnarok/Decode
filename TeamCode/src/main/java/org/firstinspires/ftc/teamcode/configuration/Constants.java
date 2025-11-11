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
			.mass(11.0)
			.forwardZeroPowerAcceleration(-36.0)
			.lateralZeroPowerAcceleration(-57.0)
			
			.translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.075))
			.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.015, 0.03))
			
			.headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.02, 0.055))
			.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.6, 0, 0.05, 0.03))
			
			.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5, 0, 0.001, 0.6, 0.15))
			.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.2, 0, 0.0003, 0.6, 0.05))
			
			.useSecondaryDrivePIDF(true)
			.useSecondaryHeadingPIDF(true)
			.useSecondaryTranslationalPIDF(true);
	
	public static final MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.leftFrontMotorName(Settings.HardwareIDs.FRONT_LEFT_MOTOR)
			.leftRearMotorName(Settings.HardwareIDs.REAR_LEFT_MOTOR)
			.rightFrontMotorName(Settings.HardwareIDs.FRONT_RIGHT_MOTOR)
			.rightRearMotorName(Settings.HardwareIDs.REAR_RIGHT_MOTOR)
			.leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
			.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
			.xVelocity(64.0).yVelocity(52);
	
	public static PinpointConstants localizerConstants = new PinpointConstants()
			.forwardPodY(6)
			.strafePodX(-6)
			.distanceUnit(DistanceUnit.INCH)
			.hardwareMapName(Settings.HardwareIDs.PINPOINT)
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
	
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
			0.998,
			0.01,
			0.01,
			0.001,
			50,
			1.2,
			10,
			1
	);
	
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