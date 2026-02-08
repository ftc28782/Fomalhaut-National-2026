package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DT Motor Test - LeftFront", group = "Debug")
public class DTLeftFrontPower1 extends OpMode {

    private DcMotorEx leftFront;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
    }

    @Override
    public void loop() {
        leftFront.setPower(1.0);

        telemetry.addData("Motor", "leftFront");
        telemetry.addData("Power", 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (leftFront != null) leftFront.setPower(0.0);
    }
}