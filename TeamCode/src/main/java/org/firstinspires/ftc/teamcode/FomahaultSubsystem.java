package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.concurrent.TimeUnit;

public class FomahaultSubsystem{
    boolean hasVision;
    Limelight3A limelightChassis;
    double TargetVelocity = 3500;
    double TicksPerRev = 28;
    double camX;
    double camY;
    Follower follower;
    double alphaXY = 0.2;
    double hood_position = 0.585;
    SunriseRobot robot;
    CRServo servo1, servo2 = null;
    DcMotorEx shooter1, shooter2, intake = null;
    Servo hoodservo = null;
    double servoPower;
    final double GOAL_BLUE_X = -63, GOAL_BLUE_Y = -64;
    double c = 0;
    Deadline IMUTimer;
    double turretAngle;
    double encoder;
    double IMUDegress;
    PIDController turretPID;
    double P = 0.014;
    double D = 0.0015;
    double fusedX;
    double fusedY;
    double distance;
    IMU imu;
    double angleToGoal;
    double shooter_power;
    double error;
    public void Inicialize(HardwareMap hardwareMap) {

        //TURRET PID
        turretPID = new PIDController(P, 0.0, D);
        turretPID.setTolerance(0.5);
        turretPID.setSetPoint(0);

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        IMUTimer = new Deadline(500, TimeUnit.MILLISECONDS);

        //SERVO
        servo1 = hardwareMap.get(CRServo.class, "turretLeft");
        servo2 = hardwareMap.get(CRServo.class, "rightTurret");

        //PINPOINT
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0,0,0));
        robot = new SunriseRobot(SunriseRobot.OpModeType.TELEOP, hardwareMap);

        //LIMELIGHT
        limelightChassis = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelightChassis.pipelineSwitch(0);
        limelightChassis.start();

        //MOTORS
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodservo = hardwareMap.get(Servo.class, "hoodServo");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(7,0,0,14.2);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public void Run() {

        //THROUGHBORE ENCODER
        encoder = (intake.getCurrentPosition() / 77.369);
        turretAngle = encoder;

        //IMU
        if (IMUTimer.hasExpired() && c < 1) {
            imu.resetYaw();
            IMUTimer.reset();
            c = c + 1;
        }
        IMUDegress = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        follower.update();
        // PINPOINT
        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();

        // LIMELIGHT
//        double turretRadius = 7;
//        double robotX = camX - (turretRadius * Math.cos(Math.toRadians(turretAngle)));
//        double robotY = camY - (turretRadius * Math.sin(Math.toRadians(turretAngle)));
//        limelightChassis.updateRobotOrientation(turretAngle);
        limelightChassis.updateRobotOrientation(IMUDegress);

        hasVision = false;

        LLResult resultTurret = limelightChassis.getLatestResult();
        if (resultTurret != null && resultTurret.isValid()) {
            Pose3D camPose3D = resultTurret.getBotpose_MT2();
            if (camPose3D != null) {
                camX = camPose3D.getPosition().x * 39.3701;
                camY = camPose3D.getPosition().y * 39.3701;
                hasVision = true;
            }
        }

        //ALPLHA FILTER & DISTANCE
        fusedX = odoX;
        fusedY = odoY;

        double errorVision = Math.hypot(camX - odoX, camY - odoY);
        if (hasVision && errorVision > 0.5) {
            fusedX = odoX * (1 - alphaXY) + camX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + camY * alphaXY;
        }
        follower.setPose(new Pose(fusedX,fusedY,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        double dx = GOAL_BLUE_X - fusedX;
        double dy = GOAL_BLUE_Y - fusedY;

        distance = Math.hypot(dx, dy);

        angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - IMUDegress - turretAngle);
        if (Math.abs(angleToGoal) < 0.4) {
            angleToGoal = 0;
        }
        servoPower = -turretPID.calculate(angleToGoal);

        if (turretAngle > 135 && servoPower > 0) {
            servoPower = 0;
        }
        if (turretAngle < -135 && servoPower < 0) {
            servoPower = 0;
        }
        servo1.setPower(servoPower);
        servo2.setPower(servoPower);

        turretPID.setPID(P, 0, D);

        //SHOOTER AND HOOD POSITION
        hood_position = -0.8909748 + 0.0521592 * distance - 0.0006677889 * Math.pow(distance, 2) + 0.000003639036 * Math.pow(distance, 3) - 7.141361e-9 * Math.pow(distance, 4);
        hoodservo.setPosition(hood_position); //Set Hood position

        shooter_power = (TargetVelocity * TicksPerRev / 60);
//        TargetVelocity = 1746.163 + 12.32215 * distance - 0.005318002 * Math.pow(distance, 2);

        double Shooter1Vel = (shooter1.getVelocity() * 60 / TicksPerRev);
        double Shooter2Vel = (shooter2.getVelocity() * 60 / TicksPerRev);
        double CurrentVelocity = (Shooter1Vel + Shooter2Vel) / 2;
        error = TargetVelocity - CurrentVelocity;
    }
}
