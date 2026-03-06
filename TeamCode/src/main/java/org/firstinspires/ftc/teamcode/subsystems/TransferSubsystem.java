package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Transfer subsystem that controls the transfer servo for moving game pieces.
 */
public class TransferSubsystem extends SubsystemBase {

    private final Servo transferServo;
    private double currentPower = 0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(Servo.class, "transferServo");
    }

    public void setPosition(double position) {
        currentPower = position;
        transferServo.setPosition(position);
    }

    /**
     * Run the transfer forward.
     */
    public void runForward() {
        setPosition(0);
    }

    /**
     * Run the transfer backward.
     */
    public void runBackward() {
        setPosition(-1);
    }

    /**
     * Stop the transfer servo.
     */
    public void stop() { setPosition(0.8); }

    /**
     * Get the current power.
     */
    public double getCurrentPower() {
        return currentPower;
    }
}
