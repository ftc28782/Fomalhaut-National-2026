package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Intake subsystem that controls the intake motor.
 * Note: The intake motor is also used as the turret encoder source.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private double currentPower = 0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set the intake motor power.
     * @param power Power to set (-1 to 1)
     */
    public void setPower(double power) {
        currentPower = power;
        intakeMotor.setPower(power);
    }

    /**
     * Run the intake forward.
     */
    public void runForward() {
        setPower(1);
    }

    /**
     * Run the intake backward (outtake).
     */
    public void runBackward() {
        setPower(1);
    }

    /**
     * Stop the intake.
     */
    public void stop() {
        setPower(0);
    }

    /**
     * Get the current power.
     */
    public double getCurrentPower() {
        return currentPower;
    }

    /**
     * Get the intake motor for encoder access (used by turret subsystem).
     */
    public DcMotorEx getMotor() {
        return intakeMotor;
    }

    /**
     * Get the encoder position.
     */
    public int getEncoderPosition() {
        return intakeMotor.getCurrentPosition();
    }
}
