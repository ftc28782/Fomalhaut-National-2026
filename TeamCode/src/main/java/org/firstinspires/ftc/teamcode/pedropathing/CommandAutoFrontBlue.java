package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.ShootForTimeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;

/**
 * Autonomous using a manual state machine with the Command-based structure.
 */
@Autonomous(name = "Command Auto Front Blue")
public class CommandAutoFrontBlue extends CommandOpMode {

    private Robot robot;
    private AutoPaths paths;
    private boolean stateInit, arrived = false;
    private Timer pathTimer;
    private int pathState = 0;
    public Pose startPose = new Pose(40.000, 136.000, Math.toRadians(270));

    @Override
    public void initialize() {
        // Initialize robot, timer, and paths
        robot = new Robot(hardwareMap,startPose);
        pathTimer = new Timer();
        paths = new AutoPaths();

        // Set up the turret to track the goal continuously
        robot.turret.setDefaultCommand(
                new TurretTrackCommand(
                        robot.turret,
                        robot::getAngleToGoal,
                        () -> 0 // No manual turret input in auto
                )
        );
    }

    @Override
    public void run() {
        // Essential updates for robot sensors and commands
        robot.update();
        CommandScheduler.getInstance().run();

        // Run the manual state machine
        statePathUpdate();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Distance to Goal", robot.getDistanceToGoal());
        telemetry.update();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case 0: // Score 1 (StartShot1)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShot1);
                    stateInit = true;
                }
                if (!robot.follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(1);
                }
                break;

            case 1: // Coleta 1 (Intake1)
                if (!stateInit) {
                    robot.follower.followPath(paths.toIntake1);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) setPathState(2);
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        stateInit = false;
        arrived = false;
        pathTimer.resetTimer();
    }

    /**
     * Inner class containing all autonomous paths, starting from the robot's initial pose.
     */
    private class AutoPaths {
        public final PathChain toShot1;
        public final PathChain toIntake1;

        public AutoPaths() {
            toShot1 = robot.follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,
                                    new Pose(55.066, 88.296)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(137))

                    .build();

            toIntake1 = robot.follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.066, 88.296),
                                    new Pose(41.837, 83.486),
                                    new Pose(16.810, 83.300)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

        }
}
}
