package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DT Motor Test - RightFront", group = "Debug")
public class DTRightFrontPower1 extends OpMode {

    private DcMotorEx rightFront;

    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
    }

    @Override
    public void loop() {
        rightFront.setPower(1.0);

        telemetry.addData("Motor", "rightFront");
        telemetry.addData("Power", 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (rightFront != null) rightFront.setPower(0.0);
    }
}