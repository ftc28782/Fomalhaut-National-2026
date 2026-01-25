package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;


@TeleOp (name = "DebugRobot")
public class DebugRobot extends OpMode { //Simple robot for tests

    Limelight3A limelightTurret;
    DcMotor intake, shooter1, shooter2 = null;
    CRServo servo2 = null;
    Servo hoodservo = null;
    Follower follower;
    GamepadEx driver;
    SunriseRobot robot;
    private double Sinal = -1;
    private String SinalAtual = "-";
    private double shooter_power = 1;
    private double hood_position = 0.48;
    private double intake_power = 1;
    private double camX = 0;
    private double camY = 0;
    private double camH = 0;
    private double turretRadius = 7.0;
    private final double GOAL_BLUE_X = -63, GOAL_BLUE_Y = -64;

    @Override
    public void init(){
        driver = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        robot = new SunriseRobot(SunriseRobot.OpModeType.TELEOP, hardwareMap);
        follower.startTeleopDrive();

        limelightTurret = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelightTurret.start();


        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        servo2 = hardwareMap.get(CRServo.class, "turretLeft");
        hoodservo = hardwareMap.get(Servo.class, "hoodServo");

    }

    @Override
    public void loop(){

        LLResult resultTurret = limelightTurret.getLatestResult();
        if (resultTurret != null && resultTurret.isValid()) {
            Pose3D camPose3D = resultTurret.getBotpose();
            if (camPose3D != null) {
                 camX = camPose3D.getPosition().x * 39.3701;
                 camY = camPose3D.getPosition().y * 39.3701;
                 camH = camPose3D.getOrientation().getYaw(AngleUnit.DEGREES);
                 turretRadius = 7.0;
            }
        }
        double robotX = camX - (turretRadius * Math.cos(Math.toRadians(camH)));
        double robotY = camY - (turretRadius * Math.sin(Math.toRadians(camH)));

        double dx = GOAL_BLUE_X - robotX;
        double dy = GOAL_BLUE_Y - robotY;

        double distance = Math.hypot(dx, dy);



        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("Distancia", distance);
        telemetry.addLine("B para alternar entre + e -");
        telemetry.addData("Sinal atual:", SinalAtual);
        telemetry.addData("Y para alterar a direção do Hood", hood_position);
        telemetry.addData("X para alterar a potência do Shooter", shooter_power);
        telemetry.addData("Botão esquerdo para alterar a potência do intake", intake_power);
        telemetry.addLine("Obs: Se o shooter estiver invertido inverta o sinal da potência do shooter");
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
            shooter_power = shooter_power + (Sinal * 0.05);
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
            shooter1.setPower(shooter_power);
            shooter2.setPower(shooter_power);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
        hoodservo.setPosition(hood_position); //Set Hood position
    }
}