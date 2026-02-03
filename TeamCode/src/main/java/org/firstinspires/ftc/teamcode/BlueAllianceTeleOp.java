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
 * Functionally equivalent to the original DebugLLChassis.
 */
@TeleOp(name = "Blue Alliance Teleop (Commands)")
public class BlueAllianceTeleOp extends CommandOpMode {

    private Robot robot;
    private GamepadEx driver;

    // Blue goal coordinates
    private static final double BLUE_GOAL_X = -66;
    private static final double BLUE_GOAL_Y = 66;

    @Override
    public void initialize() {
        // Initialize robot and gamepad
        robot = new Robot(hardwareMap);
        driver = new GamepadEx(gamepad1);

        // Set blue goal coordinates
        robot.setGoal(BLUE_GOAL_X, BLUE_GOAL_Y);

        // Start teleop driving
        robot.startTeleOpDrive();

        // Set up turret tracking as the default command
        robot.turret.setDefaultCommand(
                new TurretTrackCommand(
                        robot.turret,
                        robot::getAngleToGoal,
                        () -> gamepad1.right_stick_x
                )
        );

        // Button bindings
        // X button - Toggle flywheel
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new FlywheelToggleCommand(robot.flywheel));

        // A button - Shoot (intake + transfer when flywheel at speed)
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new ShootCommand(robot.intake, robot.transfer, robot.flywheel));

        // B button - Force feed (transfer only)
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new ForceFeedCommand(robot.intake, robot.transfer));

        // Left trigger - Intake only (while held)
        // Note: Triggers are handled in the run() method since they're analog
    }

    @Override
    public void run() {
        // Update robot state (vision, localization, etc.)
        robot.update();

        // Update flywheel target velocity based on distance
        if (robot.flywheel.isRunning()) {
            double distance = robot.getDistanceToGoal();
            robot.flywheel.setVelocityForDistance(distance);
        }

        // Handle driving
        robot.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        // Handle left trigger intake (analog input, not bound to command)
        boolean triggerIntake = gamepad1.left_trigger > 0.1;
        boolean shooting = gamepad1.a && robot.flywheel.isAtTargetVelocity(250);
        boolean forceFeed = gamepad1.b;

        // Only run intake via trigger if not already being controlled by shoot/forcefeed commands
        if (triggerIntake && !shooting && !forceFeed) {
            robot.intake.setPower(1);
        } else if (!shooting && !forceFeed) {
            robot.intake.stop();
        }

        // Run the command scheduler
        CommandScheduler.getInstance().run();

        // Telemetry
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("X LL", robot.getCamX());
        telemetry.addData("Y LL", robot.getCamY());
        telemetry.addData("FusedX", robot.getFusedX());
        telemetry.addData("FusedY", robot.getFusedY());
        telemetry.addData("Distance", robot.getDistanceToGoal());
        telemetry.addData("Shooter Error", robot.flywheel.getVelocityError());
        telemetry.addData("Target Velocity", robot.flywheel.getTargetVelocityRPM());
        telemetry.addData("IMUAngle", robot.getIMUYawDegrees());
        telemetry.addData("Turret Angle", robot.turret.getCurrentAngle());
        telemetry.addData("Angle to Goal", robot.getAngleToGoal());
        telemetry.addData("Hood", robot.flywheel.getHoodPosition());
        telemetry.addData("Flywheel Running", robot.flywheel.isRunning());
        telemetry.update();
    }
}
