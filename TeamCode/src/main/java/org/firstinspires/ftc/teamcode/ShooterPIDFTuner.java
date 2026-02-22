package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "ShooterPIDFTuner")
public class ShooterPIDFTuner extends OpMode {

    public DcMotorEx shooter1,shooter2;
    public double FullSpeed = 3600; //rpm
    public double HalfSpeed = 1500; //rpm
    public double TicksPerRev = 28;
    double TargetVelocity = HalfSpeed;
    double I = 0;
    double P = 0;
    double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,0,0);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
            I += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            I -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,0,0);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooter1.setVelocity(TargetVelocity * TicksPerRev / 60);
        shooter2.setVelocity(TargetVelocity * TicksPerRev / 60);

        double Shooter1Vel = (shooter1.getVelocity() * 60 / TicksPerRev);
        double Shooter2Vel = (shooter2.getVelocity() * 60 / TicksPerRev);
        double CurrentVelocity = (Shooter1Vel + Shooter2Vel) / 2;
        double error = TargetVelocity - CurrentVelocity;

        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", CurrentVelocity);
        telemetry.addData("Shooter1 RPM","%.2f",Shooter1Vel);
        telemetry.addData("Shooter2 RPM","%.2f",Shooter2Vel);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("------------------------------------------");
        telemetry.addData("P","%.5f (D-Pad U/D)",P);
        telemetry.addData("I","%.5f (D-Pad L/R)",I);
        telemetry.addData("Step Size","%.4f",stepSizes[stepIndex]);
        telemetry.addLine("B - StepSize Switch");
        telemetry.update();


    }
}