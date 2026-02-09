package org.firstinspires.ftc.teamcode;

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
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.concurrent.TimeUnit;


//LAST REDDEBUGCHASSIS BEFORE SETPOWER CALIBRATION
@TeleOp(name = "Red Alliance Teleop")
public class DebugREDChassis extends OpMode {
    private boolean hasVision;
    Limelight3A limelightChassis;
    public double TargetVelocity = 3500;
    public double TicksPerRev = 28;
    private double camX;
    private double camY;
    Follower follower;
    double alphaXY = 0.2;
    private double hood_position = 0.5;
    GamepadEx driver;
    DcMotorEx shooter1, shooter2, intake, turretMotor = null;
    Servo hoodservo = null;
//    Servo transferServo = null;
    private double turretPower;
    private final double GOAL_BLUE_X = -66, GOAL_BLUE_Y = 66;
    private double c = 0;
    Deadline IMUTimer;
    private double turretAngle;
    private double encoder;
    private double IMUDegress;
    PIDFController turretPID;
    private double P = 6;
    private double I = 0.7;
    private double tP = 0.014;
    private double tD = 0.0015;
    public int stepIndex = 1;
    private IMU imu;
    private double a;
    private double transferPower;
    private double intakePower;
    private boolean PDchange;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, 0, 0);

    public void init() {

        //TURRET PID
        turretPID = new PIDFController(tP, 0.0, tD, 0);
        turretPID.setTolerance(0);
        turretPID.setSetPoint(0);

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        IMUTimer = new Deadline(500, TimeUnit.MILLISECONDS);

        //TURRET MOTOR
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        //TRANSFER SERVO
//        transferServo = hardwareMap.get(Servo.class, "transferServo");

        //PINPOINT
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

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
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void loop() {

        // UPDATE DRIVER INPUTS
        driver.readButtons(); // Process WasPressed events

        //THROUGHBORE ENCODER
        encoder = (intake.getCurrentPosition() / 73.40501792085333);
        turretAngle = encoder;

        //IMU
        if (IMUTimer.hasExpired() && c < 1) {
            imu.resetYaw();
            IMUTimer.reset();
            c = c + 1;
        }
        IMUDegress = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false, 1.5708);

        // PINPOINT
        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();

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
        if (hasVision && errorVision > 3) {
            fusedX = odoX * (1 - alphaXY) + camX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + camY * alphaXY;
            follower.setPose(new Pose(fusedX, fusedY, (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))));
        }
        double dx = GOAL_BLUE_X - fusedX;
        double dy = GOAL_BLUE_Y - fusedY;

        double distance = Math.hypot(dx, dy);

        double angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - IMUDegress - turretAngle);
        if (Math.abs(angleToGoal) < 0.4) {
            angleToGoal = 0;
        }
        if (PDchange) {
            turretPID.setPIDF(tP, 0.0, tD, 0);
        }
        turretPower = -turretPID.calculate(angleToGoal);


        if (turretAngle > 115 && turretPower > 0) {
            turretPower = 0;
        }
        if (turretAngle < -115 && turretPower < 0) {
            turretPower = 0;
        }
        turretMotor.setPower(turretPower);

        //PIDF CALIBRATOR
        double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        PDchange = false;
        if (gamepad1.dpadLeftWasPressed()) {
            tD += stepSizes[stepIndex];
             PDchange = false;
        }
        if (gamepad1.dpadRightWasPressed()) {
            tD -= stepSizes[stepIndex];
             PDchange = false;
        }
        if (gamepad1.dpadUpWasPressed()) {
            tP += stepSizes[stepIndex];
             PDchange = false;
        }
        if (gamepad1.dpadDownWasPressed()) {
            tP -= stepSizes[stepIndex];
             PDchange = false;
        }

        pidfCoefficients = new PIDFCoefficients(P, I, 0, 0);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //SHOOTER AND HOOD POSITION
//        hood_position = -0.8909748 + 0.0521592 * distance - 0.0006677889 * Math.pow(distance, 2) + 0.000003639036 * Math.pow(distance, 3) - 7.141361e-9 * Math.pow(distance, 4);
        hoodservo.setPosition(hood_position); //Set Hood position
        if (gamepad1.leftBumperWasPressed()) {
            hood_position -= 0.01;
        } else if (gamepad1.rightBumperWasPressed()) {
            hood_position += 0.01;
        }

        TargetVelocity = 1364.45 + 21.20203 * distance - 0.03188598 * Math.pow(distance, 2);
//        if (gamepad1.dpadDownWasPressed()) {
//            TargetVelocity = TargetVelocity - 100;
//        }
//        if (gamepad1.dpadUpWasPressed()) {
//            TargetVelocity = TargetVelocity + 100;
//    }

    double shooter_power = (TargetVelocity * TicksPerRev / 60);

    double Shooter1Vel = (shooter1.getVelocity() * 60 / TicksPerRev);
    double Shooter2Vel = (shooter2.getVelocity() * 60 / TicksPerRev);
    double CurrentVelocity = (Shooter1Vel + Shooter2Vel) / 2;
    double error = TargetVelocity - CurrentVelocity;

    boolean intakeByTrigger = gamepad1.left_trigger > 0.1;
    boolean intakeByA = gamepad1.a && Math.abs(error) < 250;

        if(intakeByA ||gamepad1.b)

    {
        transferPower = 0.3;
    } else

    {
        transferPower = 0;
    }
        if(intakeByTrigger ||intakeByA)

    {
        intakePower = -1;
    } else

    {
        intakePower = 0;
    }
        if(intakeByTrigger ||intakeByA ||gamepad1.b)

    {
        intake.setPower(intakePower);
//        transferServo.setPosition(transferPower);
    } else

    {
        intake.setPower(0);
//        transferServo.setPosition(0);
    }
        if(gamepad1.xWasPressed()) { //Shooter
        a++;
    }
        if(a %2==1)

    {
        shooter1.setVelocity(shooter_power);
        shooter2.setVelocity(shooter_power);
    } else

    {
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }
        telemetry.addData("P","%.5f (D-Pad U/D)",P);
        telemetry.addData("I","%.5f (D-Pad L/R)",I);
        telemetry.addData("Step Size","%.4f",stepSizes[stepIndex]);
        telemetry.addLine("B - StepSize Switch");
        telemetry.addData("X LL",camX);
        telemetry.addData("Y LL",camY);
        telemetry.addData("FusedX",fusedX);
        telemetry.addData("FusedY",fusedY);
        telemetry.addData("Distance",distance);
        telemetry.addData("Shooter Error",error);
        telemetry.addData("Target Velocity",TargetVelocity);
        telemetry.addData("IMUAngle",IMUDegress);
        telemetry.addData("Turret Angle",turretAngle);
        telemetry.addData("Angle to Goal",angleToGoal);
        telemetry.addData("tP","%.5f (D-Pad U/D)",tP);
        telemetry.addData("tD","%.5f (D-Pad L/R)",tD);
        telemetry.addData("Hood",hood_position);
        telemetry.update();
}
}