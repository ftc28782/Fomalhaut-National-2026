package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "DebugLLChassis")
public class DebugLLChassis extends OpMode {

    private boolean hasVision;
    Limelight3A limelightChassis;
    private double camX = 0;
    private double camY = 0;
    Follower follower;
    double alphaXY = 0.2;
    GamepadEx driver;
    SunriseRobot robot;
    Servo servo1 = null;
    private final double GOAL_BLUE_X = -58.346457, GOAL_BLUE_Y = -55.629921;
    private double rightAngle = 0.55;
    private double leftAngle = 0.52;
    private ElapsedTime timer;
    private double c = 0;
    Deadline IMUTimer;
    Servo servo2 = null;
    private IMU imu;
    public void init() {
        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        IMUTimer = new Deadline(500, TimeUnit.MILLISECONDS);

        //SERVO
        servo1 = hardwareMap.get(Servo.class, "turretLeft");
        servo2 = hardwareMap.get(Servo.class, "rightTurret");
        timer = new ElapsedTime();

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
    }
    public void loop(){
        //IMU

        if (IMUTimer.hasExpired() && c < 1) {
            imu.resetYaw();
            IMUTimer.reset();
           c = c + 1;
        }
        double IMUDegress = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // PINPOINT

        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();
        double odoH = followerPose.getHeading();
        double odoHdegrees = Math.toDegrees(odoH);


        // LIMELIGHT
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
        if (hasVision && errorVision > 2) {
            fusedX = odoX * (1 - alphaXY) + camX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + camY * alphaXY;
            follower.setPose(new Pose(fusedX,fusedY,odoH));
        }
        double dx = GOAL_BLUE_X - fusedX;
        double dy = GOAL_BLUE_Y - fusedY;

        double distance = Math.hypot(dx, dy);

        //SERVO POSISTION

         if (gamepad1.leftBumperWasPressed()) {
             leftAngle = leftAngle + 0.01;
             rightAngle = rightAngle + 0.01;
             timer.reset();
         }

         if (gamepad1.rightBumperWasPressed()) {
             rightAngle = rightAngle - 0.01;
             leftAngle = leftAngle - 0.01;
             timer.reset();
         }
         double servoAngle = (rightAngle + leftAngle) /2;

         //COLCOCAR COM BASE NISSO UMA EQUACAO LINEAR, ONDE EM UMA POSICAO VAI SER 0 GRAUS (RETO),
        // OUTRA POSICAO É 90°, OUTRA -90°, ETC.... USANDO MYGRAPHFIT. COM ISSO VAI DAR PARA FAZER O YAW NA LIMELIGHT DA TURRET, POIS VAMOS TER O ANGULO
        servo1.setPosition(leftAngle);
        servo2.setPosition(rightAngle);

        telemetry.addData("X LL", camX);
        telemetry.addData("Y LL", camY);
        telemetry.addData("Heading", odoHdegrees);
        telemetry.addData("OdoX", odoX);
        telemetry.addData("OdoY", odoY);
        telemetry.addData("FusedX", fusedX);
        telemetry.addData("FusedY", fusedY);
        telemetry.addData("Distance", distance);
        telemetry.addData("IMUAngle", IMUDegress);
        telemetry.addData("Servo Position", servoAngle);
        telemetry.update();
    }
}
