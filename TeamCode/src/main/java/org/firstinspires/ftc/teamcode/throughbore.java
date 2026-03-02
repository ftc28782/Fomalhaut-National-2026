package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "ThroughBore Encoder Test", group = "Debug")
public class throughbore extends OpMode {
//terminar o ngc do throughbore
    private DcMotorEx encoder;
    @Override
    public void init() {
    encoder = hardwareMap.get(DcMotorEx.class, "backLeft");
    }

    @Override
    public void loop() {
    telemetry.addData("Encoder Position", encoder.getCurrentPosition());
    }
}
