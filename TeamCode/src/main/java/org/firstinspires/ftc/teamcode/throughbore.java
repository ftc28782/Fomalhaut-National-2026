package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class throughbore extends OpMode {
//terminar o ngc do throughbore
    private DcMotorEx encoder;
    @Override
    public void init() {
    encoder = hardwareMap.get(DcMotorEx.class, "encoder");
    }

    @Override
    public void loop() {
    telemetry.addData("Encoder Position", encoder.getCurrentPosition());
    }
}
