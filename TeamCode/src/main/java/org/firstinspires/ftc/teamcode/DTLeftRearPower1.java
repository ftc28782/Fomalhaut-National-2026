package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotorEx;

    @TeleOp(name = "DT Motor Test - LeftRear", group = "Debug")
    public class DTLeftRearPower1 extends OpMode {

        private DcMotorEx leftRear;

        @Override
        public void init() {
            leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        }

        @Override
        public void loop() {
            leftRear.setPower(1.0);

            telemetry.addData("Motor", "leftRear");
            telemetry.addData("Power", 1.0);
            telemetry.update();
        }

        @Override
        public void stop() {
            if (leftRear != null) leftRear.setPower(0.0);
        }
    }