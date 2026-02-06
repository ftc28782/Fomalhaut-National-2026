package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(5)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.018, 0.035))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.0,0.0003,0.6,0.0))
            .centripetalScaling(0.0005)
            .forwardZeroPowerAcceleration(-42.901)
            .lateralZeroPowerAcceleration(-67.8464);

    public static MecanumConstants driveConstants = new MecanumConstants()
    .maxPower(1)
    .leftFrontMotorName("frontLeft")
    .leftRearMotorName("backLeft")
    .rightFrontMotorName("frontRight")
    .rightRearMotorName("backRight")
    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .useBrakeModeInTeleOp(true)
    .xVelocity(60.755)
    .yVelocity(48.707); //mudar uns 2 inches na configuracao da limelight


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(-5.75)
            .strafePodX(-1)
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
//            .forwardTicksToInches()
//            .strafeTicksToInches()
            .forwardEncoder_HardwareMapName("X")
            .strafeEncoder_HardwareMapName("Y")
            .IMU_HardwareMapName("imu")
                .IMU_Orientation(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
              RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
//BEFORE TWO WHEEL CONSTANTS