package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "intake")
public class simpleintake extends OpMode {
    DcMotorEx intake, intakeencoder, s1, s2 = null;
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intakeencoder = hardwareMap.get(DcMotorEx.class, "frontLeft");
        s1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        s2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
    }
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(1);
        }
        if (gamepad1.b) {
            intake.setPower(-1);
        }
        // HD Hex 9:1 = 252 ticks por revolução na saída (28 * 9)
        // Velocidade vem em ticks/segundo, converter para RPM: (vel / 252) * 60
        double rpm = (intakeencoder.getVelocity() / 252.0) * 60.0;
        telemetry.addData("rpm", rpm);
        if (gamepad1.right_trigger > 0.1) {
            s1.setPower(1);
        }
        if (gamepad1.left_trigger > 0.1) {
            s1.setPower(-1);
        }
    }
}
