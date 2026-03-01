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
    private double startHeading = Math.toRadians(0);
    public Pose startPose = new Pose(110.22637106184365, 136, startHeading);

    @Override
    public void initialize() {

        // Initialize robot, timer, and paths
        robot = new Robot(hardwareMap,startPose);
        pathTimer = new Timer();
        paths = new AutoPaths();

        //Set alliance
        robot.setAlliance(Robot.Alliance.AUTO_RED);

        // Deixa a flywheel ligada 100% do tempo
        robot.flywheel.setDefaultCommand(
                new FlywheelRunCommand(robot.flywheel, robot)
        );
        // Also explicitly schedule the flywheel command to ensure it starts immediately
        // (some command schedulers require an explicit schedule to kick off default-like behavior)
        telemetry.addData("Auto", "Scheduled FlywheelRunCommand");
        // Set up the turret to track the goal continuously
        robot.turret.setDefaultCommand(
                new TurretTrackCommand(
                        robot.turret,
                        robot.turret::getCurrentAngle
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

        robot.flywheel.setVelocityForDistance(robot.getDistanceToGoal());
        Intake();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X LL", robot.getCamX());
        telemetry.addData("Y LL", robot.getCamY());
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Heading", robot.getHeadingDegrees());
        telemetry.addData("Turret Angle", robot.turret.getCurrentAngle());
        telemetry.addData("error", robot.flywheel.getVelocityError());
        telemetry.addData("Distance to Goal", robot.getDistanceToGoal());
        telemetry.update();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case 0: // Score 1 (StartShot1)
                robot.transfer.stop();
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot1);
                    stateInit = true;
                }
                ShootLogic(1);
                break;

            case 1: // Coleta 1 (Intake1)
                if (!stateInit) {
                    robot.follower.followPath(paths.Intake1);
                    stateInit = true;
                    Intake();
                }
                if (!robot.follower.isBusy()) {
                    setPathState(2);
                    endIntake();
                }
                break;

            case 2: // Volta para atirar 2 (toShoot2)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot2);
                    stateInit = true;
                }
                ShootLogic(3);
                break;

            case 3: // Coleta 2 (Intake2)
                if (!stateInit) {
                    robot.follower.followPath(paths.Intake2);
                    stateInit = true;
                    Intake();
                }
                if (!robot.follower.isBusy()) {
                    setPathState(4);
                    endIntake();
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
                ShootLogic(20);
                break;

            case 20:
                if (!stateInit) {
                    robot.follower.followPath(paths.Path10);
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
                    Intake();
                }
                if (!robot.follower.isBusy()) {
                    setPathState(7);
                    endIntake();
                }
                break;

            case 7: // Volta para atirar 4 (toShoot4)
                if (!stateInit) {
                    robot.follower.followPath(paths.toShoot4);
                    stateInit = true;
                }
                ShootLogic(8);
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

    public void ShootLogic(int newState) {
        if (!robot.follower.isBusy() && !arrived) {
            arrived = true;
            pathTimer.resetTimer();
        }
        if (arrived) {
            Shoot();
        }
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.6) {
            endShoot();
            pathState = newState;
            stateInit = false;
            arrived = false;
            pathTimer.resetTimer();
        }
    }
    public void Shoot() {
        robot.intake.setPower(1);
        if (robot.flywheel.isAtTargetVelocity() && !robot.follower.isBusy()) {
            robot.transfer.runForward();
        } else {
            // Se a velocidade cair (ex: após o primeiro disco sair), ele para e espera recuperar
            robot.transfer.stop();
        }
    }
    public void endShoot() {
        robot.transfer.stop();
    }

    public void Intake() {
        robot.intake.setPower(1);
    }
    public void endIntake() {
        robot.intake.setPower(1);
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
        public PathChain Path10;
        public PathChain Intake3;
        public PathChain toShoot4;
        public PathChain EndPoint;

        public AutoPaths() {
            toShoot1 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.226, 136.000),
                                    new Pose(94.383, 87.990)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))
                    .build();

            Intake1 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.383, 87.990),
                                    new Pose(96.878, 82.819),
                                    new Pose(123.640, 83.919)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            toShoot2 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.640, 83.919),
                                    new Pose(92.503, 86.281)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))
                    .build();

            Intake2 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.503, 86.281),
                                    new Pose(92.492, 55.743),
                                    new Pose(126.525, 59.136)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Gate1 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.525, 59.136),
                                    new Pose(110.612, 66.680),
                                    new Pose(125.525, 65.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            toShoot3 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.525, 65.000),
                                    new Pose(92.867, 88.557)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(51))
                    .build();

            Path10 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.867, 88.557),
                                    new Pose(98.805, 43.564)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))
                    .build();

            Intake3 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(98.805, 43.564),
                                    new Pose(104.782, 33.224),
                                    new Pose(126.919, 34.668)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            toShoot4 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.919, 34.668),
                                    new Pose(93.682, 86.884)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))
                    .build();

            EndPoint = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(93.682, 86.884),
                                    new Pose(115.831, 72.161)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();
        }
}
}
