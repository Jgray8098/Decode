package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.6)
            .forwardZeroPowerAcceleration(-33.73407296)
            .lateralZeroPowerAcceleration(-70.42644157)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.65, 0, 0.002, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5,0.0,0.002,0.6,0.02))
            .centripetalScaling(0.0005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontMotor")
            .rightRearMotorName("RightBackMotor")
            .leftRearMotorName("LeftBackMotor")
            .leftFrontMotorName("LeftFrontMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(68.16958479)
            .yVelocity(53.58226982);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.001993996)
            .strafeTicksToInches(0.001998945)
            .turnTicksToInches(0.001977165)
            .leftPodY(6.91)
            .rightPodY(-6.91)
            .strafePodX(-1.132)
            .leftEncoder_HardwareMapName("LeftFrontMotor")
            .rightEncoder_HardwareMapName("RightFrontMotor")
            .strafeEncoder_HardwareMapName("LeftBackMotor")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.7,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();

    }
}