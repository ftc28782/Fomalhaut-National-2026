package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@TeleOp (name = "DebugRobot")
public class DebugRobot extends OpMode { //Simple robot for tests

    Pose pinpointPose;
    Follower follower;
    public double TargetVelocity = 3500; //rpm
    public double TicksPerRev = 28;
    Limelight3A limelightTurret;
    DcMotor intake = null;
    DcMotorEx shooter1, shooter2;
    CRServo servo2 = null;
    Servo hoodservo = null;
    GamepadEx driver;
    SunriseRobot robot;
    private double Sinal = -1;
    private String SinalAtual = "-";
    private double hood_position = 0.805;
    private double intake_power = 1;
    private double camX = 0;
    private double camY = 0;
    private double camH = 0;
    private double turretRadius = 7.0;
    Pose2D fusedPose;

    double alphaXY = 0.25;
    double alphaH  = 0.15;
    Pose2D newPose;
    private final double GOAL_BLUE_X = -58.346457, GOAL_BLUE_Y = -55.629921;

    private final double TICKS_PER_REV = 8192.0;
    private final double GEAR_RATIO = 3.4;
    private final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private double turretAngle;
    private DcMotorEx encoder;
    private boolean hasVision;


    @Override
    public void init(){
        driver = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0,0,0));
        newPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        robot = new SunriseRobot(SunriseRobot.OpModeType.TELEOP, hardwareMap);
        follower.startTeleopDrive();

        limelightTurret = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelightTurret.pipelineSwitch(0);
        limelightTurret.start();


        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        servo2 = hardwareMap.get(CRServo.class, "turretLeft");
        hoodservo = hardwareMap.get(Servo.class, "hoodServo");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(7,0,0,14.2);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop(){

        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        newPose = PoseConverter.poseToPose2D(follower.getPose(), InvertedFTCCoordinates.INSTANCE);

//        turretAngle = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
turretAngle = newPose.getHeading(AngleUnit.DEGREES);
        limelightTurret.updateRobotOrientation(turretAngle);

        hasVision = false;

        LLResult resultTurret = limelightTurret.getLatestResult();
        if (resultTurret != null && resultTurret.isValid()) {
            Pose3D camPose3D = resultTurret.getBotpose_MT2();
            if (camPose3D != null) {
                 camX = camPose3D.getPosition().x * 39.3701;
                 camY = camPose3D.getPosition().y * 39.3701;
                 camH = turretAngle;
                 turretRadius = 7.0;
                hasVision = true;
            }
        }
        double turretHeadingRad = Math.toRadians(newPose.getHeading(AngleUnit.DEGREES) + turretAngle);
        double robotX = camX - (turretRadius * Math.cos(Math.toRadians(turretHeadingRad)));
        double robotY = camY - (turretRadius * Math.sin(Math.toRadians(turretHeadingRad)));

        double shooter_power = (TargetVelocity * TicksPerRev / 60);

        double Shooter1Vel = (shooter1.getVelocity() * 60 / TicksPerRev);
        double Shooter2Vel = (shooter2.getVelocity() * 60 / TicksPerRev);
        double CurrentVelocity = (Shooter1Vel + Shooter2Vel) / 2;
        double error = TargetVelocity - CurrentVelocity;

        double odoX = newPose.getX(DistanceUnit.INCH);
        double odoY = newPose.getY(DistanceUnit.INCH);
        double odoH = newPose.getHeading(AngleUnit.DEGREES);

        double fusedX = odoX;
        double fusedY = odoY;

        double errorVision = Math.hypot(robotX - odoX, robotY - odoY);
        if (hasVision && errorVision > 1) {
            fusedX = odoX * (1 - alphaXY) + robotX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + robotY * alphaXY;
            fusedPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, odoH);
            follower.setPose(new Pose(fusedX,fusedY,newPose.getHeading(AngleUnit.RADIANS)));
        }
        double dx = GOAL_BLUE_X - fusedX;
        double dy = GOAL_BLUE_Y - fusedY;

        double distance = Math.hypot(dx, dy);

        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", CurrentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Distância", distance);
        telemetry.addLine("B para alternar entre + e -");
        telemetry.addData("Sinal atual:", SinalAtual);
        telemetry.addData("Y para alterar a direção do Hood", hood_position);
        telemetry.addData("X para alterar a potência do Shooter", TargetVelocity);
        telemetry.addData("Botão esquerdo para alterar a potência do intake", intake_power);
        telemetry.addLine("Obs: Se o shooter estiver invertido inverta o sinal da potência do shooter");
        telemetry.addData("X LL", camX);
        telemetry.addData("Y LL", camY);
        telemetry.addData("Heading", newPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Turret Angle", encoder.getCurrentPosition());
        telemetry.update();

        if (Sinal < 0) {
            SinalAtual = "-";
        } else {
            SinalAtual = "+";
        }

        if (gamepad1.bWasPressed()) { //Signal alternation
            Sinal = Sinal * -1;
        }

        if (gamepad1.yWasPressed()){ //Hood calibration
            hood_position = hood_position + (Sinal * 0.025);
        }
        if (gamepad1.xWasPressed()) { //Shooter calibration
            TargetVelocity = TargetVelocity + (Sinal * 150);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            intake_power = intake_power + (Sinal * 0.05);
        }

        if (gamepad1.left_trigger > .1) { //Intake
            intake.setPower(intake_power);
        } else {
            intake.setPower(0);
        }

        double turretPower = 0;

        if (gamepad1.left_bumper) {
            turretPower = 1;
        } else if (gamepad1.right_bumper) {
            turretPower = -1;
        }

        servo2.setPower(turretPower);


        if (gamepad1.right_trigger > .1) { //Shooter
            shooter1.setVelocity(shooter_power);
            shooter2.setVelocity(shooter_power);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }
        hoodservo.setPosition(hood_position); //Set Hood position
    }
}