package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.FlywheelToggleCommand;
import org.firstinspires.ftc.teamcode.commands.ForceFeedCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;

/**
 * Command-based TeleOp for Blue Alliance.
 */
@TeleOp(name = "Blue Alliance Teleop (Commands)")
public class BlueAllianceTeleOp extends CommandOpMode {

    private Robot robot;
    private GamepadEx driver;

    // Blue goal coordinates

    @Override
    public void initialize() {
        // Initialize robot and gamepad
        robot = new Robot(hardwareMap);
        driver = new GamepadEx(gamepad1);

        // Set alliance and goal
        robot.setAlliance(Robot.Alliance.BLUE);

        // Start teleop driving
        robot.startTeleOpDrive();

        // Set up turret tracking as the default command
        robot.turret.setDefaultCommand(
                new TurretTrackCommand(
                        robot.turret,
                        robot::getAngleToGoal
                )
        );
                // Usando SolversLib GamepadEx
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {robot.updateVision();});

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new FlywheelToggleCommand(robot.flywheel));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new ShootCommand(robot.intake, robot.transfer, robot.flywheel));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new ForceFeedCommand(robot.intake, robot.transfer));
    }

    @Override
    public void run() {
        robot.update();

        robot.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x);

        boolean triggerIntake = gamepad1.left_trigger > 0.1;
        boolean shootingIntake = gamepad1.right_trigger > 0.1;
        boolean shooting = gamepad1.right_trigger > 0.1 && robot.flywheel.isAtTargetVelocity();


        if (shooting) {
            robot.transfer.runForward();
            } else {
            robot.transfer.stop();
        }
        if (triggerIntake || shootingIntake) {
            robot.intake.setPower(1);
        } else {
            robot.intake.stop();
        }

        CommandScheduler.getInstance().run();
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("X LL", robot.getCamX());
        telemetry.addData("Y LL", robot.getCamY());
        telemetry.addData("X", robot.getX());
        telemetry.addData("Y", robot.getY());
        telemetry.addData("X Velocity", robot.follower.getVelocity().getXComponent());
        telemetry.addData("Y Velocity", robot.follower.getVelocity().getYComponent());
        telemetry.addData("Distance", robot.getDistanceToGoal());
        telemetry.addData("Shooter Error", robot.flywheel.getVelocityError());
        telemetry.addData("Target Velocity", robot.flywheel.getTargetVelocityRPM());
        telemetry.addData("Heading", robot.getHeadingDegrees());
        telemetry.addData("Turret Angle", robot.turret.getCurrentAngle());
        telemetry.addData("Angle to Goal", robot.getAngleToGoal());
        telemetry.update();
    }
}
