package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "ShooterPIDFTuner")
public class ShooterPIDFTuner extends OpMode {

    public DcMotorEx shooter;
    public double FullSpeed = 312; //rpm
    public double HalfSpeed = 156; //rpm
    public double TicksPerRev = 537.7;
    double TargetVelocity = FullSpeed;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }
    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (TargetVelocity == FullSpeed) {
                TargetVelocity = HalfSpeed;
            } else {
                TargetVelocity = FullSpeed;
            }
        }
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                F += stepSizes[stepIndex];
            }
            if (gamepad1.dpadRightWasPressed()) {
                F -= stepSizes[stepIndex];
            }
            if (gamepad1.dpadUpWasPressed()) {
                P += stepSizes[stepIndex];
            }
            if (gamepad1.dpadDownWasPressed()) {
                P -= stepSizes[stepIndex];
            }
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            shooter.setVelocity(TargetVelocity * TicksPerRev / 60);

            double CurrentVelocity = (shooter.getVelocity() * 60 / TicksPerRev);
            double error = TargetVelocity - CurrentVelocity;

            telemetry.addData("Target Velocity", TargetVelocity);
            telemetry.addData("Current Velocity", "%.2f", CurrentVelocity);
            telemetry.addData("Error","%.2f",error);
            telemetry.addLine("------------------------------------------");
            telemetry.addData("P","%.5f (D-Pad U/D)",P);
            telemetry.addData("F","%.5f (D-Pad L/R)",F);
            telemetry.addData("Step Size","%.4f",stepSizes[stepIndex]);
            telemetry.addLine("B - StepSize Switch");
            telemetry.update();


    }
}