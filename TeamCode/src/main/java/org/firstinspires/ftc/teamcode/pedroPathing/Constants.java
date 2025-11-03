package org.firstinspires.ftc.teamcode.pedroPathing;

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
import org.firstinspires.ftc.teamcode.configuration.Settings;

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
			.mass(10.5)
			.forwardZeroPowerAcceleration(-36.0)
			.lateralZeroPowerAcceleration(-57.0)
			
			.translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.007, 0.075))
			.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
			
			.headingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.05, 0.06))
			.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.8, 0, 0.07, 0.03))
			
			.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.00035, 0.6, 0.015))
			.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01))
			
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
			.xVelocity(53.2).yVelocity(43.3);
	
	public static PinpointConstants localizerConstants = new PinpointConstants()
			.forwardPodY(6)
			.strafePodX(-5.5)
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
			2,
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