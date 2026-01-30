package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TEST_frontRight")
public class TEST_frontRight extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "frontRight");
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
