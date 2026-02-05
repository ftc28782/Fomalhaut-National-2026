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
import org.firstinspires.ftc.teamcode.commands.FlywheelRunCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelToggleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.ShootForTimeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;

/**
 * Autonomous using a manual state machine with the Command-based structure.
 */
@Autonomous(name = "Command Auto Front Blue")
public class CommandAutoFrontBlue extends CommandOpMode {

    private Robot robot;
    private AutoPaths paths;
    private IntakeRunCommand intake;
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

        //Set alliance
        robot.setAlliance(Robot.Alliance.BLUE);

        // Deixa a flywheel ligada 100% do tempo
        robot.flywheel.setDefaultCommand(
                new FlywheelRunCommand(robot.flywheel, robot)
        );

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
        telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X LL", robot.getCamX());
        telemetry.addData("Y LL", robot.getCamY());
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Distance to Goal", robot.getDistanceToGoal());
        telemetry.update();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case 0: // Score 1 (StartShot1)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot1);
                    stateInit = true;
                }
                if (!robot.follower.isBusy() && !arrived) {
                    arrived = true;
                    pathTimer.resetTimer();
                }
                if (arrived) {
                    Shoot();
                }
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(1);
                    endShoot();
                }
                break;

            case 1: // Coleta 1 (Intake1)
                if (!stateInit) {
                    robot.follower.followPath(paths.Intake1);
                    stateInit = true;
                    enableIntake();
                }
                if (!robot.follower.isBusy()) {
                    setPathState(2);
                    disableIntake();
                }

            case 2: // Volta para atirar 2 (toShoot2)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot2);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3: // Coleta 2 (Intake2)
                if (!stateInit) {
                    robot.follower.followPath(paths.Intake2);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4: // Gate 1 (Gate1)
                if (!stateInit) {
                    robot.follower.followPath(paths.Gate1);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5: // Volta para atirar 3 (toShoot3)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot3);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // Coleta 3 (Intake3)
                if (!stateInit) {
                    robot.follower.followPath(paths.Intake3);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7: // Volta para atirar 4 (toShoot4)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot4);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8: // Ponto final (EndPoint)
                if (!stateInit) {
                    robot.follower.followPath(paths.EndPoint);
                    stateInit = true;
                }
                if (!robot.follower.isBusy()) {
                    setPathState(9);
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

    // liga (chama quando apertar botão)
    /**
     * Lógica inteligente de tiro: só liga o intake/transfer se a flywheel estiver pronta.
     */
    public void Shoot() {
        if (robot.flywheel.isAtTargetVelocity(250)) {
            robot.intake.setPower(1);
            robot.transfer.setPower(1);
        } else {
            // Se a velocidade cair (ex: após o primeiro disco sair), ele para e espera recuperar
            robot.intake.stop();
            robot.transfer.stop();
        }
    }
    public void endShoot() {
        robot.intake.stop();
        robot.transfer.stop();
    }

    public void enableIntake() {
        if (intake == null) {
            intake = new IntakeRunCommand(robot.intake, 1.0); // power = 1.0
        }
        if (!intake.isScheduled()) {
            intake.schedule();
        }
    }

    // desliga (chama quando soltar botão)
    public void disableIntake() {
        if (intake != null && intake.isScheduled()) {
            intake.cancel(); // chama end() do comando
        } else {
            robot.intake.stop(); // garantia: para o motor mesmo sem comando
        }
    }

    /**
     * Inner class containing all autonomous paths, starting from the robot's initial pose.
     */
    private class AutoPaths {
        public PathChain toShoot1;
        public PathChain Intake1;
        public PathChain toShoot2;
        public PathChain Intake2;
        public PathChain Gate1;
        public PathChain toShoot3;
        public PathChain Intake3;
        public PathChain toShoot4;
        public PathChain EndPoint;

        public AutoPaths() {
            toShoot1 = robot.follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,

                                    new Pose(52.162, 91.283)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                    .build();

            Intake1 = robot.follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(52.162, 91.283),
                                    new Pose(47.122, 82.819),
                                    new Pose(17.360, 83.919)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            toShoot2 = robot.follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.360, 83.919),

                                    new Pose(52.994, 90.173)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                    .build();

            Intake2 = robot.follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(52.994, 90.173),
                                    new Pose(51.508, 55.743),
                                    new Pose(10.610, 58.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Gate1 = robot.follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.610, 58.800),
                                    new Pose(17.664, 61.741),
                                    new Pose(14.068, 67.332)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                    .build();

            toShoot3 = robot.follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.068, 67.332),

                                    new Pose(53.827, 89.064)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                    .build();

            Intake3 = robot.follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.827, 89.064),
                                    new Pose(37.034, 72.206),
                                    new Pose(10.048, 34.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            toShoot4 = robot.follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.048, 34.500),

                                    new Pose(54.659, 88.231)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            EndPoint = robot.follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.659, 88.231),

                                    new Pose(22.751, 71.861)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();
        }
}
}
