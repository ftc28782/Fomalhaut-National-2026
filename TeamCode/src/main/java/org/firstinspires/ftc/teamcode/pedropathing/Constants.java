package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(7)
        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0))
            .centripetalScaling(0.005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
    .maxPower(1)
    .leftFrontMotorName("frontLeft")
    .leftRearMotorName("backLeft")
    .rightFrontMotorName("frontRight")
    .rightRearMotorName("backRight")
    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4)
            .strafePodX(1)
            .distanceUnit(DistanceUnit.INCH)

            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1)
    ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
