package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.DebugLLChassis;
import org.firstinspires.ftc.teamcode.SunriseRobot;

@Autonomous(name = "Auto Back Red")
public class AutoBackRed extends OpMode {

    Follower follower;
    private Timer pathTimer, opModeTimer;
    public int pathState = 0;
    private boolean stateInit, arrived = false;
    private static String Phrase = "Nada";
    private Paths paths;
    public void statePathUpdate() {
        switch (pathState) {

            case 0:
                if (!stateInit) {
                    follower.followPath(paths.Path1);
                    Phrase = "Shot 1";
                    stateInit = true;
                }

                if (!follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        stateInit = false;
        arrived = false;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(87.626, 8.187, Math.toRadians(180)));
        paths = new Paths(follower);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("State", pathState);
        telemetry.addData("Action", Phrase);
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    public static class Paths {
        public PathChain Path1;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.626, 8.187),

                                    new Pose(110.226, 14.708)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();
        }
    }
}

