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


@Autonomous
public class PedroAutonomous extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public int pathState = 0;

    private Paths paths;

    public static class Paths {
        public PathChain StartShot1;
        public PathChain ToIntake1;
        public PathChain Intake1;
        public PathChain Return1Shot2;
        public PathChain Go1;
        public PathChain Return2Shot3;
        public PathChain Go2;
        public PathChain Return3Shot4;
        public PathChain Go3;
        public PathChain Return4Shot5;
        public PathChain ToIntake2;
        public PathChain Intake2;
        public PathChain Return5Shot6;
        public PathChain ToIntake3;
        public PathChain Intake3;
        public PathChain Return6Shot7;
        public PathChain EndPoint;

        public Paths(Follower follower) {
            StartShot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.261, 136),

                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            ToIntake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 87.000),
                                    new Pose(57.524, 68.226),
                                    new Pose(45.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 60.000),

                                    new Pose(8.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Return1Shot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.000, 60.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Go1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 87.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(12.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))

                    .build();

            Return2Shot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.000, 61.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))

                    .build();

            Go2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 87.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(12.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))

                    .build();

            Return3Shot4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.000, 61.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))

                    .build();

            Go3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 87.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(12.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))

                    .build();

            Return4Shot5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.000, 61.000),
                                    new Pose(54.000, 54.000),
                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))

                    .build();

            ToIntake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 87.000),

                                    new Pose(45.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 84.000),

                                    new Pose(16.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Return5Shot6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.000, 84.000),

                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            ToIntake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 87.000),

                                    new Pose(41.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Intake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.000, 36.000),

                                    new Pose(9.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Return6Shot7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.000, 36.000),

                                    new Pose(56.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            EndPoint = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 87.000),

                                    new Pose(40.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();
        }
    }
    public void statePathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.StartShot1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToIntake1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Return1Shot2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Go1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Return2Shot3);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Go2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Return3Shot4);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Go3);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Return4Shot5);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToIntake2);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake2);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Return5Shot6);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToIntake3);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake3);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Return6Shot7);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(paths.EndPoint);
                }
                setPathState(17);

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
        Pose startPose = new Pose(32.261, 136, Math.toRadians(180));
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