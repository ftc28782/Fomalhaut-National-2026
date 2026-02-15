package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "intake")
public class simpleintake extends OpMode {
    DcMotorEx intake, intakeencoder = null;
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intakeencoder = hardwareMap.get(DcMotorEx.class, "frontLeft");
    }
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(1);
        }
        if (gamepad1.b) {
            intake.setPower(-1);
        }
        // HD Hex 4:1 = 112 ticks por revolução na saída
        // Velocidade vem em ticks/segundo, converter para RPM: (vel / 112) * 60
        double rpm = (intakeencoder.getVelocity() / 112.0) * 60.0;
        telemetry.addData("rpm", rpm);
    }
}
