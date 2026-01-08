package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    GamepadEx driver = new GamepadEx(gamepad1);
    SunriseRobot robot;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();

        robot = new SunriseRobot(SunriseRobot.OpModeType.TELEOP, hardwareMap);

        follower.startTeleopDrive();
    }

    @Override
    public void run() {
        super.run();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}