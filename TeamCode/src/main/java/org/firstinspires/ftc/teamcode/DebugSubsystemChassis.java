package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DebugSubsystemChassis")
public class DebugSubsystemChassis extends OpMode {
    FomahaultSubsystem fomahault;
    private int stepIndex = 1;
    private double a;

    public void init() {

        fomahault = new FomahaultSubsystem();
        fomahault.Inicialize(hardwareMap);

    }
    public void loop(){

        follower.update();
        fomahault.Run();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        //TURRET SERVOS
        double chassisTurn = gamepad1.right_stick_x;

        fomahault.servoPower = fomahault.servoPower + (chassisTurn * 1);

        //PIDF CALIBRATOR
        double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            fomahault.D += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            fomahault.D -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            fomahault.P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            fomahault.P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            fomahault.TargetVelocity = fomahault.TargetVelocity + 100;
        } else if (gamepad1.dpadDownWasPressed()) {
            fomahault.TargetVelocity = fomahault.TargetVelocity - 100;
        }

        boolean intakeByTrigger = gamepad1.left_trigger > 0.1;
        boolean intakeByA = gamepad1.a && Math.abs(fomahault.error) < 150;

        if (intakeByTrigger || intakeByA) {
            fomahault.intake.setPower(1);
        } else {
            fomahault.intake.setPower(0);
        }

        if (gamepad1.xWasPressed()) { //Shooter
            a++;
            if (a % 2 != 1) {
                fomahault.shooter1.setVelocity(fomahault.shooter_power);
                fomahault.shooter2.setVelocity(fomahault.shooter_power);
            } else {
                fomahault.shooter1.setVelocity(0);
                fomahault.shooter2.setVelocity(0);
            }
        }

        telemetry.addData("P", "%.5f (D-Pad U/D)", fomahault.P);
        telemetry.addData("D", "%.5f (D-Pad L/R)", fomahault.D);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine("B - StepSize Switch");
        telemetry.addData("X LL", fomahault.camX);
        telemetry.addData("Y LL", fomahault.camY);
        telemetry.addData("FusedX", fomahault.fusedX);
        telemetry.addData("FusedY", fomahault.fusedY);
        telemetry.addData("Distance", fomahault.distance);
        telemetry.addData("Shooter Error", fomahault.error);
        telemetry.addData("Target Velocity", fomahault.TargetVelocity);
        telemetry.addData("IMUAngle", fomahault.IMUDegress);
        telemetry.addData("Turret Angle", fomahault.turretAngle);
        telemetry.addData("Angle to Goal", fomahault.angleToGoal);
        telemetry.update();
    }
}