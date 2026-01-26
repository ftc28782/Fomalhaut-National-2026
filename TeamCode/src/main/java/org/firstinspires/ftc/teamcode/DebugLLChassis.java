package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@TeleOp(name = "DebugLLChassis")
public class DebugLLChassis extends OpMode {

    private boolean hasVision;
    private double camX = 0;
    private double camY = 0;
    Follower follower;
    double alphaXY = 0.25;
    GamepadEx driver;
    SunriseRobot robot;
    private final double GOAL_BLUE_X = -58.346457, GOAL_BLUE_Y = -55.629921;
    Limelight3A limelightChassis;
    public void init() {
        //PINPOINT
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0,0,0));
        robot = new SunriseRobot(SunriseRobot.OpModeType.TELEOP, hardwareMap);

        //TELEOP\\\\
        driver = new GamepadEx(gamepad1);
        follower.startTeleopDrive();


        //LIMELIGHT
        limelightChassis = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelightChassis.pipelineSwitch(0);
        limelightChassis.start();
    }
    public void loop(){
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // PINPOINT

        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();
        double odoH = followerPose.getHeading();
        double odoHdegrees = Math.toDegrees(odoH);


        // LIMELIGHT

        limelightChassis.updateRobotOrientation(odoHdegrees);

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

        //ALPLHA FILTER
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

        telemetry.addData("X LL", camX);
        telemetry.addData("Y LL", camY);
        telemetry.addData("Heading", odoH);
        telemetry.addData("OdoX", odoX);
        telemetry.addData("OdoY", odoY);
        telemetry.addData("FusedX", fusedX);
        telemetry.addData("FusedY", fusedY);
        telemetry.addData("Distance", distance);
        telemetry.update();
    }
}
