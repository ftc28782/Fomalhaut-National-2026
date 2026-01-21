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

@Autonomous(name = "PedroAutonomous")
public class PedroAutonomous extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public int pathState = 0;
    private boolean stateInit = false;

    private static double StartX = 30.3;
    private static double StartY = 132.5;

    private static String Phrase = "Nada";

    private Paths paths;

    public static class Paths {
        public PathChain StartShot1, ToIntake1, Intake1, Return1Shot2;
        public PathChain Go1, Return2Shot3, Go2, Return3Shot4;
        public PathChain Go3, Return4Shot5, ToIntake2, Intake2;
        public PathChain Return5Shot6, ToIntake3, Intake3;
        public PathChain Return6Shot7, EndPoint;

        public Paths(Follower follower) {

            StartShot1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(StartX, StartY),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

            ToIntake1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56, 87),
                            new Pose(57.524, 68.226),
                            new Pose(45, 58.5)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            Intake1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(45, 58.5),
                            new Pose(8, 58.5)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Return1Shot2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(8, 60),
                            new Pose(54, 54),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

            Go1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56, 87),
                            new Pose(54, 54),
                            new Pose(12, 61)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150)).build();

            Return2Shot3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(12, 61),
                            new Pose(54, 54),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135)).build();

            Go2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56, 87),
                            new Pose(54, 54),
                            new Pose(12, 61)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150)).build();

            Return3Shot4 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(12, 61),
                            new Pose(54, 54),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135)).build();

            Go3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56, 87),
                            new Pose(54, 54),
                            new Pose(12, 61)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150)).build();

            Return4Shot5 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(12, 61),
                            new Pose(54, 54),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135)).build();

            ToIntake2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56, 87),
                            new Pose(45, 84)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Intake2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(45, 84),
                            new Pose(16, 84)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Return5Shot6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(16, 84),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

            ToIntake3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56, 87),
                            new Pose(41, 36)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            Intake3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(41, 36),
                            new Pose(9, 36)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Return6Shot7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(9, 36),
                            new Pose(56, 87)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

            EndPoint = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56, 87),
                            new Pose(42, 72)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();
        }
    }

    public void statePathUpdate() {
        switch (pathState) {

            case 0:
                if (!stateInit) {
                    follower.followPath(paths.StartShot1);
                    Phrase = "Shot 1";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(1);
                break;

            case 1:
                if (!stateInit) {
                    follower.followPath(paths.ToIntake1);
                    Phrase = "To Intake 1";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(2);


                break;

            case 2:
                if (!stateInit) {
                    follower.followPath(paths.Intake1);
                    Phrase = "Intaking 1";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(3);
                break;

            case 3:
                if (!stateInit) {
                    follower.followPath(paths.Return1Shot2);
                    Phrase = "Shot 2";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(4);
                break;

            case 4:
                if (!stateInit) {
                    Phrase = "Shooting";
                    pathTimer.resetTimer();
                    stateInit = true;
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) setPathState(5);
                break;

            case 5:
                if (!stateInit) {
                    follower.followPath(paths.Go1);
                    Phrase = "Go 1";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(6);
                break;

            case 6:
                if (!stateInit) {
                    follower.followPath(paths.Return2Shot3);
                    Phrase = "Shot 3";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(7);
                break;

            case 7:
                if (!stateInit) {
                    follower.followPath(paths.Go2);
                    Phrase = "Go 2";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(8);
                break;

            case 8:
                if (!stateInit) {
                    follower.followPath(paths.Return3Shot4);
                    Phrase = "Shot 4";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(9);
                break;

            case 9:
                if (!stateInit) {
                    follower.followPath(paths.Go3);
                    Phrase = "Go 3";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(10);
                break;

            case 10:
                if (!stateInit) {
                    follower.followPath(paths.Return4Shot5);
                    Phrase = "Shot 5";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(11);
                break;

            case 11:
                if (!stateInit) {
                    follower.followPath(paths.ToIntake2);
                    Phrase = "To Intake 2";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(12);
                break;

            case 12:
                if (!stateInit) {
                    follower.followPath(paths.Intake2);
                    Phrase = "Intaking 2";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(13);
                break;

            case 13:
                if (!stateInit) {
                    follower.followPath(paths.Return5Shot6);
                    Phrase = "Shot 6";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(14);
                break;

            case 14:
                if (!stateInit) {
                    follower.followPath(paths.ToIntake3);
                    Phrase = "To Intake 3";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(15);
                break;

            case 15:
                if (!stateInit) {
                    follower.followPath(paths.Intake3);
                    Phrase = "Intaking 3";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(16);
                break;

            case 16:
                if (!stateInit) {
                    follower.followPath(paths.Return6Shot7);
                    Phrase = "Shot 7";
                    stateInit = true;
                }
                if (!follower.isBusy()) setPathState(17);
                break;

            case 17:
                if (!stateInit) {
                    follower.followPath(paths.EndPoint);
                    Phrase = "End";
                    stateInit = true;
                }
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        stateInit = false;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(StartX, StartY, Math.toRadians(180)));
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
        telemetry.update();
    }
}
