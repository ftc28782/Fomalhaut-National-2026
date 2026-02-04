package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous(name = "IGNORE ESSE AUTONOMO12")
public class AutoFrontRed12 extends OpMode {

    Follower follower;
    private Timer pathTimer, opModeTimer;
    public int pathState = 0;
    private boolean stateInit, arrived = false;
    private static String Phrase = "Nada";
    private Paths paths;
    public void statePathUpdate() {
        switch (pathState) {
            case 0: // Score 1 (StartShot1)
                if (!stateInit) {
                    follower.followPath(paths.StartShot1);
                    Phrase = "Shot 1";
                    stateInit = true;
                }
                if (!follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(1);
                }
                break;

            case 1: // Coleta 1 (Intake1)
                if (!stateInit) {
                    follower.followPath(paths.Intake1);
                    Phrase = "Intake 1";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(2);
                break;

            case 2: // Score 2 (Return1Shot2)
                if (!stateInit) {
                    follower.followPath(paths.Return1Shot2);
                    Phrase = "Shot 2";
                    stateInit = true;
                }
                if (!follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(3);
                }
                break;

            case 3: // Coleta 2 (Intake2)
                if (!stateInit) {
                    follower.followPath(paths.Intake2);
                    Phrase = "Intake 2";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(4);
                break;

            case 4: // Score 3 (Return2Shot3)
                if (!stateInit) {
                    follower.followPath(paths.Return2Shot3);
                    Phrase = "Shot 3";
                    stateInit = true;
                }
                if (!follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(5);
                }
                break;

            case 5: // Coleta 3 (Intake3)
                if (!stateInit) {
                    follower.followPath(paths.Intake3);
                    Phrase = "Intake 3";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(6);
                break;

            case 6: // Score 4 (Return3Shot4)
                if (!stateInit) {
                    follower.followPath(paths.Return3Shot4);
                    Phrase = "Shot 4";
                    stateInit = true;
                }
                if (!follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(7);
                }
                break;

            case 7: // Estacionar (EndPoint)
                if (!stateInit) {
                    follower.followPath(paths.EndPoint);
                    Phrase = "End/Park";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(8);
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
        follower.setPose(new Pose(104.104, 136.824, Math.toRadians(270)));
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
        public PathChain StartShot1;
        public PathChain Intake1;
        public PathChain Return1Shot2;
        public PathChain Intake2;
        public PathChain Return2Shot3;
        public PathChain Intake3;
        public PathChain Return3Shot4;
        public PathChain EndPoint;

        public Paths(Follower follower) {

            StartShot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.104, 136.824),

                                    new Pose(91.838, 91.283)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(45))

                    .build();

            Intake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91.838, 91.283),
                                    new Pose(96.878, 82.819),
                                    new Pose(129.190, 83.169)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Return1Shot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.190, 83.169),

                                    new Pose(91.006, 90.173)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Intake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91.006, 90.173),
                                    new Pose(92.492, 55.743),
                                    new Pose(135.190, 58.050)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Return2Shot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.190, 58.050),
                                    new Pose(92.157, 55.907),
                                    new Pose(90.173, 89.064)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Intake3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(90.173, 89.064),
                                    new Pose(78.007, 33.032),
                                    new Pose(134.852, 34.350)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Return3Shot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.852, 34.350),

                                    new Pose(89.341, 88.231)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            EndPoint = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.341, 88.231),

                                    new Pose(121.249, 71.861)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();
            }
        }
    }

