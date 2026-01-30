package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TEST_backRight")
public class TEST_backRight extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
