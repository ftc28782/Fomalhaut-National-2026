package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FlywheelRunCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous using a manual state machine with the Command-based structure.
 */
@Autonomous(name = "Command Auto Back Blue")
public class CommandAutoBackBlue extends CommandOpMode {

    private Robot robot;
    private AutoPaths paths;
    private IntakeRunCommand intake;
    private boolean stateInit, arrived = false;
    private Timer pathTimer;
    private Timer teleopTimer;
    private int pathState = 0;
    private double startHeading = Math.toRadians(90);
    public Pose startPose = new Pose(56, 8, startHeading);
    private List<Double> routine;

    @Override
    public void initialize() {

        // Initialize robot, timer, and paths
        robot = new Robot(hardwareMap,startPose);
        pathTimer = new Timer();
        teleopTimer = new Timer();

        paths = new AutoPaths();

        //Set alliance
        robot.setAlliance(Robot.Alliance.AUTO_BLUE);

        //Data saver
        routine = new ArrayList<>();

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
        robot.flywheel.setHoodPosition(0.7);
        Intake();
        endIntake();

        if (pathState == 7 || pathState == 8 || pathState == 9){
            //File saver
            String routineString = routine.toString();
            routineString = routineString.substring(1, routineString.length() - 1);

            File file = AppUtil.getInstance().getSettingsFile("File.txt");
            ReadWriteFile.writeFile(file, routineString);

            routine.clear();
            routine.add(robot.getX());
            routine.add(robot.getY());
            routine.add(robot.getHeadingDegrees());
        }
        telemetry.addData("Saved itens", routine);


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
                    robot.follower.followPath(paths.twoIntake2);
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
                ShootLogic(9); //antes era pathState 20 aqui
                break;

//            case 20:
//                if (!stateInit) {
//                    robot.follower.followPath(paths.Path10);
//                    stateInit = true;
//                }
//                if (!robot.follower.isBusy()) {
//                    setPathState(6);
//                }
//                break;
//            case 6: // Coleta 3 (Intake3)
//                if (!stateInit) {
//                    robot.follower.followPath(paths.Intake3);
//                    stateInit = true;
//                    Intake();
//                }
//                if (!robot.follower.isBusy()) {
//                    setPathState(7);
//                    endIntake();
//                }
//                break;
//
//            case 7: // Volta para atirar 4 (toShoot4)
//                if (!stateInit) {
//                    robot.follower.followPath(paths.toShoot4);
//                    stateInit = true;
//                }
//                ShootLogic(8);
//                break;
//
//            case 8: // Ponto final (EndPoint)
//                if (!stateInit) {
//                    robot.follower.followPath(paths.EndPoint);
//                    stateInit = true;
//                }
//                if (!robot.follower.isBusy()) {
//                    setPathState(9);
//                }
//                break;

            case 9: // Estado final - robô parado, dados já salvos
                // Não faz nada, autônomo acabou
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
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {
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
        public PathChain Intake1;
        public PathChain toShoot2;
        public PathChain Intake2;
        public PathChain twoIntake2;
        public PathChain toShoot3;


        public AutoPaths() {
            Intake1 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(60.266, 38.245),
                                    new Pose(17.000, 35.131)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            toShoot2 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17.000, 35.131),
                                    new Pose(59.028, 13.076)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            Intake2 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.028, 13.076),
                                    new Pose(36.914, 12.963),
                                    new Pose(9.561, 15.356)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            twoIntake2 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.561, 15.356),
                                    new Pose(24.441, 11.851),
                                    new Pose(10.431, 9.618)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toShoot3 = robot.follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.431, 9.618),
                                    new Pose(56.736, 12.646)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();
        }
    }
}
