package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DT Motor Test - RightRear", group = "Debug")
public class DTRightRearPower1 extends OpMode {

    private DcMotorEx rightRear;
    private final double TICKS_PER_REV = 537.6; // GoBilda 312 RPM

    @Override
    public void init() {
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
    }

    @Override
    public void loop() {
        rightRear.setPower(1.0);

        double rpm = (rightRear.getVelocity() / TICKS_PER_REV) * 60.0;

        telemetry.addData("Motor", "rightRear");
        telemetry.addData("Power", 1.0);
        telemetry.addData("RPM", "%.2f", rpm);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (rightRear != null) rightRear.setPower(0.0);
    }
}