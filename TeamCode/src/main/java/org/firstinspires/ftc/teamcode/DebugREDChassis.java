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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.concurrent.TimeUnit;


//LAST REDDEBUGCHASSIS BEFORE SETPOWER CALIBRATION
@TeleOp(name = "Red Alliance Teleop")
public class DebugREDChassis extends OpMode {

    //FOLLOWER
    Follower follower;
    GamepadEx driver;

    //MOTORS AND SERVOS
    DcMotorEx shooter1, shooter2, intake, turretMotor = null;
    Servo hoodServo, transferServo = null;

    //ENCODERS AND LL
    Limelight3A limelightChassis;
    private double camX;
    private double camY;
    private double errorVision;

    // LOGICS
    public double targetVelocity = 3000; //rpm
    public double TicksPerRev = 28;
    private double turretPower;
    private double hoodPosition = 1;
    private final double GOAL_BLUE_X = -66, GOAL_BLUE_Y = 66;
    private double c = 0;
    Deadline IMUTimer;
    private double odoX, odoY;
    private double turretAngle;
    PIDFController turretPID;
    private double P = 60;
    private double I = 0.0079;
    private double tP = 0.024;
    private double tD = 0.0001;
    private double tF = 0;
    private double a = 0;
    public int stepIndex = 1;
    DcMotorEx intakeencoder = null;
    private boolean PDchange;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, 0, 0);

    public void init() {

        //TURRET PID
        turretPID = new PIDFController(tP, 0.0, tD, 0);
        turretPID.setTolerance(0);

        //TURRET MOTOR
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

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

        //INTAKE ENCODER
        intakeencoder = hardwareMap.get(DcMotorEx.class, "backLeft");

        //MOTORS

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        transferServo = hardwareMap.get(Servo.class, "transferServo");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void loop() {

        // UPDATE DRIVER INPUTS
        driver.readButtons(); // Process WasPressed events

        //THROUGHBORE ENCODER
        turretAngle = (intake.getCurrentPosition() / 100.35);

        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false, 1.5708);



        //LIMELIGHT
        limelightChassis.updateRobotOrientation(follower.getHeading());

        LLResult result = limelightChassis.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D camPose3D = result.getBotpose_MT2();
            if (camPose3D != null) {
                camX = (camPose3D.getPosition().x * 39.3701);
                camY = (camPose3D.getPosition().y * 39.3701);
                errorVision = Math.hypot(camX - follower.getPose().getX(), camY - follower.getPose().getY());
                if (gamepad1.a) {
                    follower.setPose(new Pose(camX, camY, follower.getHeading()));
                }
            }
        }

        // PINPOINT
        Pose followerPose = follower.getPose();
        odoX = followerPose.getX();
        odoY = followerPose.getY();

        //GOAL AND ANGLETOGOAL CALCULATIONS
        double dx = GOAL_BLUE_X - odoX;
        double dy = GOAL_BLUE_Y - odoY;

        double distance = Math.hypot(dx, dy);

        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(follower.getTotalHeading()) + turretAngle;
        turretPower = turretPID.calculate(angleToGoal);

        //TURRET SYSTEM
        //OBS IMPORTNATE: O SINAL DO SETPOWER É SEMPRE O MESMO DO ÂNGULO
        if (turretPower >= 0) { //F from PIDF
            turretPower = turretPower + tF;
        } else {
            turretPower = turretPower - tF;
        }

        //da pra fazer esse negocio aq:
        //lembrando que os sinais do turretPower eu n faço ideia KKKKKKKKKK

//        if (turretAngle > 115 && turretPower > 0) { //Solução boa para o giro da turret que vou aplicar dps
//            turretPower = Math.abs(turretPower);
//        }
//        if (turretAngle < -115 && turretPower < 0) {
//            turretPower = -Math.abs(turretPower);
//        }
        if (turretAngle > 115 && turretPower > 0) {
            turretPower = 0;
        }
        if (turretAngle < -45 && turretPower < 0) { //Angulo original era 115º
            turretPower = 0;
        }
        if (Math.abs(angleToGoal) < 1) {
            turretPower = 0;
        }
        turretMotor.setPower(turretPower);

        //PIDF CALIBRATOR
        double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};
        if (gamepad1.yWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (PDchange) {
            turretPID.setPIDF(tP, 0.0, tD, 0);
        }
        PDchange = false;
//        if (gamepad1.dpadLeftWasPressed()) {
//            tP += stepSizes[stepIndex];
//             PDchange = true;
//        }
//        if (gamepad1.dpadRightWasPressed()) {
//            tP -= stepSizes[stepIndex];
//             PDchange = true;
//        }
//        if (gamepad1.dpadUpWasPressed()) {
//            tF += stepSizes[stepIndex];
//             PDchange = true;
//        }
//        if (gamepad1.dpadDownWasPressed()) {
//            tF -= stepSizes[stepIndex];
//             PDchange = true;
//        }

        //SHOOTER SYSTEM
        pidfCoefficients = new PIDFCoefficients(P, I, 0, 0);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

//        TargetVelocity = 1364.45 + 21.20203 * distance - 0.03188598 * Math.pow(distance, 2);
//        if (gamepad1.dpadDownWasPressed()) {
//            TargetVelocity = TargetVelocity - 100;
//        }
//        if (gamepad1.dpadUpWasPressed()) {
//            TargetVelocity = TargetVelocity + 100;
//    }

    double shooter_power = (targetVelocity * TicksPerRev / 60);


    double Shooter1Vel = (shooter1.getVelocity() * 60 / TicksPerRev);
    double Shooter2Vel = (-shooter2.getVelocity() * 60 / TicksPerRev);
    double CurrentVelocity = (Shooter1Vel + Shooter2Vel) / 2;
    double error = targetVelocity - CurrentVelocity;

        if(gamepad1.xWasPressed()) {
            a++;
        }

        if(a %2==1) {
            shooter1.setVelocity(shooter_power);
            shooter2.setVelocity(shooter_power);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        //HOOD POSITION
        targetVelocity = 2991.758
                - 66.95706 * distance
                + 2.195716 * Math.pow(distance, 2)
                - 0.02787701 * Math.pow(distance, 3)
                + 0.0001660626 * Math.pow(distance, 4)
                - 3.770099e-7 * Math.pow(distance, 5);

        hoodPosition = 0.000123486 * Math.pow(distance, 3)
                - 0.0153796 * Math.pow(distance, 2)
                + 0.591463 * distance
                - 6.03862;

        hoodServo.setPosition(hoodPosition); //Set Hood position
        if (gamepad1.leftBumperWasPressed()) {
            hoodPosition = hoodPosition + 0.1;
        } else if (gamepad1.rightBumperWasPressed()) {
            hoodPosition = hoodPosition - 0.1;
        }

    //INTAKE AND LAUCHER SYSTEM
    boolean Intake = gamepad1.left_trigger > 0.1;
    boolean Launch = gamepad1.right_trigger > 0.1;
    boolean Transfer = Math.abs(error) < 350;

    if (Launch && Transfer) {
        transferServo.setPosition(0);
    } else {
        transferServo.setPosition(0.7);
        }
        if(Launch||Intake) {
        intake.setPower(1);
    } else {
        intake.setPower(0);
        }

        //TELEMETRIES
        telemetry.addData("P","%.5f (D-Pad U/D)",P);
        telemetry.addData("I","%.5f (D-Pad L/R)",I);
        telemetry.addData("Step Size (Y to switch)","%.4f",stepSizes[stepIndex]);
        telemetry.addData("X LL",camX);
        telemetry.addData("Y LL",camY);
        telemetry.addData("FusedX",odoX);
        telemetry.addData("FusedY",odoY);
        telemetry.addData("X speed", follower.getVelocity().getXComponent());
        telemetry.addData("Y speed", follower.getVelocity().getYComponent());
        telemetry.addData("Distance",distance);
        telemetry.addData("Shooter Error",error);
        telemetry.addData("Target Velocity",targetVelocity);
        telemetry.addData("Shooter1 RPM","%.2f",Shooter1Vel);
        telemetry.addData("Shooter2 RPM","%.2f",Shooter2Vel);
        telemetry.addData("Heading (B to reset)",follower.getHeading());
        telemetry.addData("Turret Angle",turretAngle);
        telemetry.addData("Angle to Goal",angleToGoal);
        telemetry.addData("tP","%.5f (D-Pad U/D)",tP);
        telemetry.addData("tF","%.5f (D-Pad L/R)",tF);
        telemetry.addData("Hood",hoodPosition);
        telemetry.addData("TurretPower", turretPower);
        telemetry.addData("odo error",errorVision);

        double rpm = (intakeencoder.getVelocity() / 252) * 60.0;
        telemetry.addData("rpm", rpm);

        telemetry.update();
    }
}
