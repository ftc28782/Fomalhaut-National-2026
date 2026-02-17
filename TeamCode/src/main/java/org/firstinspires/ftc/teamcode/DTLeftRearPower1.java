package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DT Motor Test - LeftRear", group = "Debug")
public class DTLeftRearPower1 extends OpMode {

    private DcMotorEx leftRear;
    private final double TICKS_PER_REV = 537.6; // GoBilda 312 RPM

    @Override
    public void init() {
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
    }

    @Override
    public void loop() {
        leftRear.setPower(1.0);

        double rpm = (leftRear.getVelocity() / TICKS_PER_REV) * 60.0;

        telemetry.addData("Motor", "leftRear");
        telemetry.addData("Power", 1.0);
        telemetry.addData("RPM", "%.2f", rpm);
        telemetry.update();
    }

        @Override
        public void stop() {
            if (leftRear != null) leftRear.setPower(0.0);
        }
    }