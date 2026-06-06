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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.113)
            .forwardZeroPowerAcceleration(-29.34215361)
            .lateralZeroPowerAcceleration(-74.0827393)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0.001, 0.035))
            .headingPIDFCoefficients(new PIDFCoefficients(0.45, 0, 0.002, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.002, 0.6, 0.025))
            .centripetalScaling(0.0005);


    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("RightFrontMotor")
        .rightRearMotorName("RightBackMotor")
        .leftRearMotorName("LeftBackMotor")
        .leftFrontMotorName("LeftFrontMotor")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(84.6613213)
        .yVelocity(68.06904931);

    
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.496)
            .strafePodX(-0.013)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
