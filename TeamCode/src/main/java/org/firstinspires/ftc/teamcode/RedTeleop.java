package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RED Alliance Teleop")
public class RedTeleop extends OpMode {

    //FOLLOWER
    Follower follower;
    GamepadEx driver;

    //MOTORS AND SERVOS
    DcMotorEx shooter1, shooter2, intake, turretMotor = null;
    Servo hoodServo, transferServo = null;

    //ENCODERS AND LL
    Limelight3A limelightChassis;
    private double camX;
    private double heading;
    private double camY;
    private double errorVision;

    // LOGICS
    public double targetVelocity = 3000; //rpm
    public double TicksPerRev = 28;
    private double turretPower;
    private double hoodPosition = 1;
    private final double GOAL_RED_X = -65, GOAL_RED_Y = 64.3;
    Deadline IMUTimer;
    private double odoX, odoY;
    private double turretAngle;
    PIDFController turretPID;
    private double P = 270;
    private double F = 4;
    private double tP = 0.034;
    private double tD = 0.0003;
    private double tF = 0.06;
    private double a = 0;
    DcMotorEx g,h,i,j;

    private double xGoalOffset;
    private double yGoalOffset;
    private double distance;
    public int stepIndex = 1;
    DcMotorEx intakeencoder = null;
    private boolean PDchange;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

    // --- Teleop smoothing (linear interpolation) ---
    // target values come directly from sticks; current values are smoothed towards target each loop
    private double targetDriveForward = 0.0; // corresponds to forward/back (left stick Y)
    private double targetDriveStrafe = 0.0;  // corresponds to left/right (left stick X)
    private double targetDriveRotate = 0.0;  // corresponds to rotation (right stick X)

    private double currentDriveForward = 0.0;
    private double currentDriveStrafe = 0.0;
    private double currentDriveRotate = 0.0;

    // smoothing factor in (0,1]; closer to 1 -> faster response, closer to 0 -> smoother/slower
    // You can tweak this value to taste. Example: 0.15 is reasonably smooth but responsive.
    private double driveLerpFactor = 0.4;

    // ====== DADOS CARREGADOS DO AUTÔNOMO ======
    private boolean poseLoaded = false;
    private double loadedX, loadedY, loadedHeading;
    private double savedX, savedY, savedHeading;

    private List<Double> routine;

    public void init() {

        //TURRET PID
        turretPID = new PIDFController(tP, 0.0, tD, 0);
        turretPID.setTolerance(0);

        //TURRET MOTOR
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        //PINPOINT
        follower = Constants.createFollower(hardwareMap);

        //File Loader
        routine = new ArrayList<>();

        File closeAuto = closeAuto = AppUtil.getInstance().getSettingsFile("File.txt");
        String[] types = ReadWriteFile.readFile(closeAuto).trim().split(",");

        for (String type : types) {
            routine.add(Double.parseDouble(type));
        }

        loadedX = routine.get(0);
        loadedY = routine.get(1);
        loadedHeading = routine.get(2);

        // Se tem posição salva (não é 0,0,0), usa ela. Senão, começa em 0,0,0.
        if (loadedX != 0 || loadedY != 0 || loadedHeading != 0) {

            savedX = -(loadedY - 72);
            savedY = loadedX - 72;
            loadedHeading = loadedHeading + 90;
            loadedHeading = loadedHeading % 360;
            if (loadedHeading > 180) loadedHeading -= 360;
            if (loadedHeading < -180) loadedHeading += 360;
          follower.setPose(new Pose(savedX, savedY, Math.toRadians(loadedHeading)));

            poseLoaded = true;
        } else {
            poseLoaded = false;
        }
        follower.setPose(new Pose(0, 43, Math.toRadians(90)));

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
        g = hardwareMap.get(DcMotorEx.class,"frontLeft");
        h = hardwareMap.get(DcMotorEx.class,"backLeft");
        i = hardwareMap.get(DcMotorEx.class,"frontRight");
        j = hardwareMap.get(DcMotorEx.class,"backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        transferServo = hardwareMap.get(Servo.class, "transferServo");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Initialize smoothing state from a neutral starting point
        currentDriveForward = 0.0;
        currentDriveStrafe = 0.0;
        currentDriveRotate = 0.0;
        targetDriveForward = 0.0;
        targetDriveStrafe = 0.0;
        targetDriveRotate = 0.0;
    }

    public void loop() {
        // UPDATE DRIVER INPUTS
        driver.readButtons(); // Process WasPressed events

        //THROUGHBORE ENCODER
        turretAngle = (intake.getCurrentPosition() / 100.35);

        follower.update();

        // --- Read raw joystick targets ---
        targetDriveForward = -gamepad1.left_stick_y; // forward/back
        targetDriveStrafe = -gamepad1.left_stick_x;  // strafe
        targetDriveRotate = -gamepad1.right_stick_x; // rotation

        // --- Smooth (linear interpolate) current values towards targets ---
        currentDriveForward = lerp(currentDriveForward, targetDriveForward, driveLerpFactor);
        currentDriveStrafe  = lerp(currentDriveStrafe, targetDriveStrafe, driveLerpFactor);
        currentDriveRotate  = lerp(currentDriveRotate, targetDriveRotate, driveLerpFactor);

        // Allow on-the-fly tuning of smoothing factor (LB to decrease, RB to increase)
        if (gamepad1.leftBumperWasPressed()) {
            driveLerpFactor = Math.max(0.01, driveLerpFactor - 0.05);
        }
        if (gamepad1.rightBumperWasPressed()) {
            driveLerpFactor = Math.min(1.0, driveLerpFactor + 0.05);
        }

//        // Use the smoothed values when commanding the follower teleop drive
        follower.setTeleOpDrive(currentDriveForward, currentDriveStrafe, currentDriveRotate, false, 1.5708);
//        follower.setTeleOpDrive(targetDriveForward, targetDriveStrafe, targetDriveRotate, false, 1.5708);

        //LIMELIGHT
        limelightChassis.updateRobotOrientation(Math.toDegrees(follower.getHeading()));

        LLResult result = limelightChassis.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D camPose3D = result.getBotpose_MT2();
            if (camPose3D != null) {
                camX = (camPose3D.getPosition().x * 39.3701);
                camY = (camPose3D.getPosition().y * 39.3701);
                errorVision = Math.hypot(camX - follower.getPose().getX(), camY - follower.getPose().getY());
                if (gamepad1.aWasPressed()) {
                    follower.setPose(new Pose(camX, camY, follower.getHeading()));
                }
            }
        }

        //HEADING RESET
        if (gamepad1.bWasPressed()) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
            loadedHeading = 0;
        }

        // PINPOINT
        Pose followerPose = follower.getPose();
        odoX = followerPose.getX();
        odoY = followerPose.getY();

        //SHOOTING WHILE MOVING
        double shotTime = 1;
        xGoalOffset = GOAL_RED_X - follower.getVelocity().getXComponent() * shotTime;
        yGoalOffset = GOAL_RED_Y - follower.getVelocity().getYComponent() * shotTime;

        //GOAL AND ANGLETOGOAL CALCULATIONS WITH SHOOTING WHILE MOVING
        double dx = xGoalOffset - odoX;
        double dy = yGoalOffset - odoY;

        distance = Math.hypot(dx, dy);

        heading = Math.toDegrees(follower.getHeading());

        double angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - heading + turretAngle);
        turretPower = turretPID.calculate(angleToGoal);

        //TURRET SYSTEM
        //OBS IMPORTNATE: O SINAL DO SETPOWER É SEMPRE O MESMO DO ÂNGULO
        if (turretPower >= 0) { //F from PIDF
            turretPower = turretPower + tF;
        } else {
            turretPower = turretPower - tF;
        }

        if (turretAngle > 50 && turretPower > 0) {
            turretPower = 0;
        }
        if (turretAngle < -75 && turretPower < 0) {
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
        if (gamepad1.dpadLeftWasPressed()) {
            tP += stepSizes[stepIndex];
             PDchange = true;
        }
        if (gamepad1.dpadRightWasPressed()) {
            tP -= stepSizes[stepIndex];
             PDchange = true;
        }
        if (gamepad1.dpadUpWasPressed()) {
            tD += stepSizes[stepIndex];
             PDchange = true;
        }
        if (gamepad1.dpadDownWasPressed()) {
            tD -= stepSizes[stepIndex];
             PDchange = true;
        }

        //SHOOTER SYSTEM
        pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

//        if (gamepad1.dpadDownWasPressed()) {
//            targetVelocity = targetVelocity - 100;
//        }
//        if (gamepad1.dpadUpWasPressed()) {
//            targetVelocity = targetVelocity + 100;
//    }

    double shooter_power = (targetVelocity * TicksPerRev / 60);


    double ShooterVel = (shooter2.getVelocity() * 60 / TicksPerRev);
    double error = targetVelocity - ShooterVel;

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


        targetVelocity = 7205.578 + (2800 - 7205.578) / Math.pow(1 + Math.pow(distance / 59.03553, 761.1621), 0.0003075716);
        hoodPosition = 1 + (0.9 - 1) / Math.pow(1 + Math.pow(distance / 69.93993, 127.7445), 33.05428);

        hoodServo.setPosition(hoodPosition); //Set Hood position
//        if (gamepad1.leftBumperWasPressed()) {
//            hoodPosition = hoodPosition + 0.1;
//        } else if (gamepad1.rightBumperWasPressed()) {
//            hoodPosition = hoodPosition - 0.1;
//        }

    //INTAKE AND LAUCHER SYSTEM
    boolean Intake = gamepad1.left_trigger > 0.1;
    boolean Launch = gamepad1.right_trigger > 0.1;
    boolean Transfer = Math.abs(error) < 250;

    if (Launch && Transfer) {
        transferServo.setPosition(0);
    } else {
        transferServo.setPosition(0.8);
        }
        if(Launch||Intake) {
        intake.setPower(1);
    } else {
        intake.setPower(0);
        }

       //TELEMETRIES
        telemetry.addData("P","%.5f (D-Pad U/D)",P);
        telemetry.addData("F","%.5f (D-Pad L/R)", F);
        telemetry.addData("Step Size (Y to switch)","%.4f",stepSizes[stepIndex]);
        telemetry.addData("X LL",camX);
        telemetry.addData("Y LL",camY);
        telemetry.addData("FusedX",odoX);
        telemetry.addData("FusedY",odoY);
        telemetry.addData("X speed", follower.getVelocity().getXComponent());
        telemetry.addData("Y speed", follower.getVelocity().getYComponent());

        // Telemetry for smoothing tuning
        telemetry.addData("driveLerpFactor","%.3f", driveLerpFactor);
        telemetry.addData("targetForward","%.3f", targetDriveForward);
        telemetry.addData("currentForward","%.3f", currentDriveForward);
        telemetry.addData("targetStrafe","%.3f", targetDriveStrafe);
        telemetry.addData("currentStrafe","%.3f", currentDriveStrafe);
        telemetry.addData("targetRotate","%.3f", targetDriveRotate);
        telemetry.addData("currentRotate","%.3f", currentDriveRotate);

        telemetry.addData("X offset", xGoalOffset);
        telemetry.addData("Y offset", yGoalOffset);
        telemetry.addData("Distance",distance);
        telemetry.addData("Shooter Error",error);
        telemetry.addData("Target Velocity",targetVelocity);
        telemetry.addData("Shooter RPM","%.2f",ShooterVel);
        telemetry.addData("Heading (B to reset)",Math.toDegrees(follower.getHeading()));
        telemetry.addData("Robot Heading",heading);
        telemetry.addData("Turret Angle",turretAngle);
        telemetry.addData("Angle to Goal",angleToGoal);
        telemetry.addData("tP","%.5f (D-Pad U/D)",tP);
        telemetry.addData("tD","%.5f (D-Pad L/R)",tD);
        telemetry.addData("Hood",hoodPosition);
        telemetry.addData("TurretPower",turretPower);
        telemetry.addData("odo error",errorVision);

        double rpm = (intakeencoder.getVelocity() / 140) * 60.0;
        telemetry.addData("rpm", rpm);

        // ====== TELEMETRY DOS DADOS CARREGADOS ======
        telemetry.addLine("--- Dados do Autônomo ---");
        if (poseLoaded) {
            telemetry.addData("Pose Carregada?", "SIM");
            telemetry.addData("Loaded X", "%.2f", savedX);
            telemetry.addData("Loaded Y", "%.2f", savedY);
            telemetry.addData("Loaded Heading (deg)", "%.2f", loadedHeading);
        } else {
            telemetry.addData("Pose Carregada?", "NAO (usando 0,0,0)");
        }
        // =============================================

        telemetry.update();
    }

    // linear interpolation helper
    private double lerp(double from, double to, double t) {
        return from + t * (to - from);
    }
}
