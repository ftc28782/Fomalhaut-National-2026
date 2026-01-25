package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.ManualTurretCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleTurretAimingModeCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class NoPinpoint extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    GamepadEx driver;
    GamepadEx operator;
    SunriseRobot robot;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        super.reset();

        robot = new SunriseRobot(SunriseRobot.OpModeType.TELEOP, hardwareMap);
        operator = new GamepadEx(gamepad2);

        // Turret Controls
        robot.turretSubsystem.setDefaultCommand(
            new ManualTurretCommand(robot.turretSubsystem, () -> operator.getRightX())
        );

        operator.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new ToggleTurretAimingModeCommand(robot.turretSubsystem));
    }

    @Override
    public void runOpMode() {
        initialize();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                robot.turretSubsystem.setAlliance(TurretSubsystem.Alliance.BLUE);
            }
            if (gamepad1.b) {
                robot.turretSubsystem.setAlliance(TurretSubsystem.Alliance.RED);
            }
            telemetryData.addData("Alliance", robot.turretSubsystem.getAlliance());
            telemetryData.addData("Status", "Inicializado. Aperte X/Triangulo para AZUL, B/Circulo para VERMELHO.");
            telemetryData.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
    @Override
    public void run() {
        super.run();

//        telemetryData.addData("X", follower.getPose().getX());
//        telemetryData.addData("Y", follower.getPose().getY());
//        telemetryData.addData("Heading", follower.getPose().getHeading());

        // Turret Telemetry
        telemetryData.addData("Turret Mode", robot.turretSubsystem.getAimingMode());
        telemetryData.addData("Turret Angle", robot.turretSubsystem.getPositionDegrees());
        telemetryData.addData("Turret Target", robot.turretSubsystem.getTargetDegrees());
        telemetryData.addData("Turret Target Visible", robot.turretSubsystem.isTargetVisible());
        telemetryData.addData("Robot Pose (Turret Est.) X", robot.turretSubsystem.getCurrentRobotPose().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
        telemetryData.addData("Robot Pose (Turret Est.) Y", robot.turretSubsystem.getCurrentRobotPose().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));

        telemetryData.update();


    }
}