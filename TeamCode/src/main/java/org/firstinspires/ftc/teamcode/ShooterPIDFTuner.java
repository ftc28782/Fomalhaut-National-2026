package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;


@TeleOp(name = "ShooterPIDFTuner")
public class ShooterPIDFTuner extends OpMode {

    public DcMotorEx shooter1,shooter2;
    public double FullSpeed = 3600; //rpm
    public double HalfSpeed = 1800; //rpm
    public double TicksPerRev = 28;
    double TargetVelocity = HalfSpeed;
    double F = 4;
    double P = 270;
    double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};
    int stepIndex = 1;
    Follower follower;
    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // PINPOINT
        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();

        //SHOOTING WHILE MOVING
        double shotTime= 1;
        double xGoalOffset = -65 - follower.getVelocity().getXComponent() * shotTime;
        double yGoalOffset = 65 - follower.getVelocity().getYComponent() * shotTime;

        //GOAL AND ANGLETOGOAL CALCULATIONS WITH SHOOTING WHILE MOVING
        double dx = xGoalOffset - odoX;
        double dy = yGoalOffset - odoY;

        double distance = Math.hypot(dx, dy);

        TargetVelocity = 7205.578 + (2800 - 7205.578) / Math.pow(1 + Math.pow(distance / 59.03553, 761.1621), 0.0003075716);
//        if (gamepad1.yWasPressed()) {
//            if (TargetVelocity == FullSpeed) {
//                TargetVelocity = HalfSpeed;
//            } else {
//                TargetVelocity = FullSpeed;
//            }
//        }
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
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooter1.setVelocity(TargetVelocity * TicksPerRev / 60);
        shooter2.setVelocity(TargetVelocity * TicksPerRev / 60);

        double Shooter2Vel = (shooter2.getVelocity() * 60 / TicksPerRev);
        double CurrentVelocity = Shooter2Vel;
        double error = TargetVelocity - CurrentVelocity;

        telemetry.addData("distance", distance);
        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", CurrentVelocity);
        telemetry.addData("Shooter2 RPM","%.2f",Shooter2Vel);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("------------------------------------------");
        telemetry.addData("P","%.5f (D-Pad U/D)",P);
        telemetry.addData("F","%.5f (D-Pad L/R)", F);
        telemetry.addData("Step Size","%.4f",stepSizes[stepIndex]);
        telemetry.addLine("B - StepSize Switch");
        telemetry.update();


    }
}