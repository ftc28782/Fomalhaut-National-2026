package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DT Motor Test - RightRear", group = "Debug")
public class DTRightRearPower1 extends OpMode {

    private DcMotorEx rightRear;

    @Override
    public void init() {
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
    }

    @Override
    public void loop() {
        rightRear.setPower(1.0);

        telemetry.addData("Motor", "rightRear");
        telemetry.addData("Power", 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (rightRear != null) rightRear.setPower(0.0);
    }
}