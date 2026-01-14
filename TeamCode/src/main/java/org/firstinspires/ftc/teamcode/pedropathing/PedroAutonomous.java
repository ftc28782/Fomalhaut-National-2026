package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

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

        public Paths(Follower follower) {
            StartShot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 136.000),

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
        }
        }
    public void statePathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(paths.ToIntake1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
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
    }
}