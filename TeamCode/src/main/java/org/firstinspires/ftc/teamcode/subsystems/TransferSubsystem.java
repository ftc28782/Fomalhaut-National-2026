package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Transfer subsystem that controls the transfer servo for moving game pieces.
 */
public class TransferSubsystem extends SubsystemBase {

    private final CRServo transferServo;
    private double currentPower = 0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
    }

    /**
     * Set the transfer servo power.
     * @param power Power to set (-1 to 1)
     */
    public void setPower(double power) {
        currentPower = power;
        transferServo.setPower(power);
    }

    /**
     * Run the transfer forward.
     */
    public void runForward() {
        setPower(1);
    }

    /**
     * Run the transfer backward.
     */
    public void runBackward() {
        setPower(-1);
    }

    /**
     * Stop the transfer servo.
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
}
