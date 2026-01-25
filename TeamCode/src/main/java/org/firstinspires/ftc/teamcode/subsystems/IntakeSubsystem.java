package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    DcMotor intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public double getIntakePower() {
        return intake.getPower();
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    @Override
    public void periodic() {

    }
}
