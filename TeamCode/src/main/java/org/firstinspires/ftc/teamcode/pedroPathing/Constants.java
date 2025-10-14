package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Constants {
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0029578264717013835) //forard tuner
            .strafeTicksToInches(-0.002988785430331746) //lateral tuner
            .turnTicksToInches(0.00295439426728707153333333333333) //turn tuner
            .leftPodY(4.09375)
            .rightPodY(-4.09375)
            .strafePodX(6.55)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("leftBack")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));


    public static FollowerConstants followerConstants = new FollowerConstants()
            .centripetalScaling(0.00002)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0, 0.06, 0.1))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.1))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.05, 0.08))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03)) //done
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.01,0.03))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .forwardZeroPowerAcceleration(-27.838855711336925)
            .lateralZeroPowerAcceleration(-55.147317082044319666666666666667)
            .mass(8.391459);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }


    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(61.774252233360931333333333333333)
            .yVelocity(50.1845976098553)
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);




}
