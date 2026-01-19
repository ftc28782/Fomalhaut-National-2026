package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;


@Autonomous(name = "PedroTest")
public class PedroTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public int pathState = 0;

    private static double StartX = 33.60151691948658 ;
    private static double StartY = 135.93465577596265;

    private Paths paths;

    public static class Paths {
        public PathChain Path1;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(StartX, StartY),

                                    new Pose(72.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();
        }
    }
    public void statePathUpdate() {
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 100) {
                        follower.followPath(paths.Path1);
                    }
                }
                break;


            default:
                telemetry.addLine("Autonomous Finished");
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(StartX, StartY, Math.toRadians(180));
        follower.setPose(startPose);
        paths = new Paths(follower);


    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("Time Elapsed", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path State", pathState);
    }
}