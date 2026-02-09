package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelRunCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeForTimeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootForTimeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;

/**
 * Command-based autonomous example.
 * Demonstrates how to integrate subsystem commands with Pedro Pathing.
 */
@Autonomous(name = "Command Auto Front Red")
public class CommandAutoFrontRed extends CommandOpMode {

    private Robot robot;
    private AutoPaths paths;

    @Override
    public void initialize() {

        robot.setAlliance(Robot.Alliance.AUTO_RED);
        // Initialize robot with starting pose
        robot = new Robot(hardwareMap, new Pose(118.070, 130.521, Math.toRadians(180)));

        // Build paths
        paths = new AutoPaths();

        while(!opModeIsActive());

        // Set up the turret tracking as default command (runs continuously)
        robot.turret.setDefaultCommand(
                new TurretTrackCommand(
                        robot.turret,
                        robot::getAngleToGoal
                )
        );

        // Schedule the autonomous sequence
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // Start flywheel and drive to first shot position
                        new ParallelCommandGroup(
                                new FlywheelRunCommand(robot.flywheel,robot),
                                new FollowPathCommand(robot.follower, paths.toShot1)
                        ),

                        // Wait a moment then shoot for 2 seconds
                        new WaitCommand(500),
                        new ShootForTimeCommand(robot.intake, robot.transfer, robot.flywheel, 2.0),

                        // Drive to intake position while keeping flywheel running
                        new ParallelCommandGroup(
                                new FlywheelRunCommand(robot.flywheel,robot),
                                new SequentialCommandGroup(
                                        new FollowPathCommand(robot.follower, paths.toIntake1),
                                        new IntakeForTimeCommand(robot.intake, 1.5)
                                )
                        ),

                        // Return to shot position
                        new ParallelCommandGroup(
                                new FlywheelRunCommand(robot.flywheel,robot),
                                new FollowPathCommand(robot.follower, paths.toShot2)
                        ),

                        // Shoot again
                        new WaitCommand(500),
                        new ShootForTimeCommand(robot.intake, robot.transfer, robot.flywheel, 2.0),

                        // Drive to park position
                        new FollowPathCommand(robot.follower, paths.toPark)
                )
        );
    }

    @Override
    public void run() {
        // Update robot state (vision, localization, etc.)
        robot.update();

        // Run the command scheduler
        CommandScheduler.getInstance().run();

        // Telemetry
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Distance to Goal", robot.getDistanceToGoal());
        telemetry.addData("Turret Angle", robot.turret.getCurrentAngle());
        telemetry.addData("Flywheel Running", robot.flywheel.isRunning());
        telemetry.addData("Flywheel Velocity", robot.flywheel.getAverageVelocityRPM());
        telemetry.update();
    }

    /**
     * Inner class containing all autonomous paths.
     */
    private class AutoPaths {
        public final PathChain toShot1;
        public final PathChain toIntake1;
        public final PathChain toShot2;
        public final PathChain toPark;

        public AutoPaths() {
            // Path to first shooting position
            toShot1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(118.070, 130.521),
                            new Pose(100, 110)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path to intake position
            toIntake1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(100, 110),
                            new Pose(80, 100)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();

            // Path back to shot position
            toShot2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(80, 100),
                            new Pose(100, 110)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();

            // Path to park
            toPark = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(100, 110),
                            new Pose(120, 90)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
