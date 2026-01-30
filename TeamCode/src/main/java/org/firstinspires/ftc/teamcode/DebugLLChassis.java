package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "DebugLLChassis")
public class DebugLLChassis extends OpMode {

    private boolean hasVision;
    Limelight3A limelightChassis;
    public double TargetVelocity = 3500;
    public double TicksPerRev = 28;
    private double camX;
    private double camY;
    Follower follower;
    double alphaXY = 0.2;
    private double hood_position = 0.585;
    GamepadEx driver;
    SunriseRobot robot;
    CRServo servo1, servo2 = null;
    DcMotorEx shooter1, shooter2, intake= null;
    Servo hoodservo = null;
    private double servoPower;
    private final double GOAL_BLUE_X = -63, GOAL_BLUE_Y = -64;
    private double c = 0;
    Deadline IMUTimer;
    private double turretAngle;
    private double encoder;
    private double IMUDegress;
    PIDController turretPID;
    private double P = 0.014;
    private double D = 0.0015;
    public int stepIndex = 1;
    private IMU imu;
    private double a;

    public void init() {

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

        //TELEOP
        driver = new GamepadEx(gamepad1);
        follower.startTeleopDrive();

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
    public void loop(){

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
        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

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
        double fusedX = odoX;
        double fusedY = odoY;

        double errorVision = Math.hypot(camX - odoX, camY - odoY);
        if (hasVision && errorVision > 0.5) {
            fusedX = odoX * (1 - alphaXY) + camX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + camY * alphaXY;
        }
        follower.setPose(new Pose(fusedX,fusedY,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        double dx = GOAL_BLUE_X - fusedX;
        double dy = GOAL_BLUE_Y - fusedY;

        double distance = Math.hypot(dx, dy);

        //TURRET SERVOS
        double chassisTurn = gamepad1.right_stick_x;

        double angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - IMUDegress - turretAngle);
        if (Math.abs(angleToGoal) < 0.4) {
            angleToGoal = 0;
        }
        servoPower = -turretPID.calculate(angleToGoal) + (chassisTurn * 1);

        if (turretAngle > 135 && servoPower > 0) {
            servoPower = 0;
        }
        if (turretAngle < -135 && servoPower < 0) {
            servoPower = 0;
        }
        servo1.setPower(servoPower);
        servo2.setPower(servoPower);

        //PIDF CALIBRATOR
        double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            D -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        turretPID.setPID(P, 0, D);

        //SHOOTER AND HOOD POSITION
        hood_position = -0.8909748 + 0.0521592 * distance - 0.0006677889 * Math.pow(distance, 2) + 0.000003639036 * Math.pow(distance, 3) - 7.141361e-9 * Math.pow(distance, 4);
        hoodservo.setPosition(hood_position); //Set Hood position

        double shooter_power = (TargetVelocity * TicksPerRev / 60);
        TargetVelocity = 1746.163 + 12.32215 * distance - 0.005318002 * Math.pow(distance, 2);

        double Shooter1Vel = (shooter1.getVelocity() * 60 / TicksPerRev);
        double Shooter2Vel = (shooter2.getVelocity() * 60 / TicksPerRev);
        double CurrentVelocity = (Shooter1Vel + Shooter2Vel) / 2;
        double error = TargetVelocity - CurrentVelocity;

        boolean intakeByTrigger = gamepad1.left_trigger > 0.1;
        boolean intakeByA = gamepad1.a && Math.abs(error) < 150;

        if (intakeByTrigger || intakeByA) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
//        if (gamepad1.right_trigger > .1) { //Shooter
//            shooter1.setVelocity(shooter_power);
//            shooter2.setVelocity(shooter_power);
//        } else {
//            shooter1.setVelocity(0);
//            shooter2.setVelocity(0);
//        }
        if (gamepad1.xWasPressed()) { //Shooter
            a++;
            if (a % 2 != 1) {
                shooter1.setVelocity(shooter_power);
                shooter2.setVelocity(shooter_power);
            } else {
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
            }
        }

        telemetry.addData("P","%.5f (D-Pad U/D)",P);
        telemetry.addData("D","%.5f (D-Pad L/R)",D);
        telemetry.addData("Step Size","%.4f",stepSizes[stepIndex]);
        telemetry.addLine("B - StepSize Switch");
        telemetry.addData("X LL", camX);
        telemetry.addData("Y LL", camY);
        telemetry.addData("FusedX", fusedX);
        telemetry.addData("FusedY", fusedY);
        telemetry.addData("Distance", distance);
        telemetry.addData("IMUAngle", IMUDegress);
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Angle to Goal", angleToGoal);
        telemetry.update();
    }
}
