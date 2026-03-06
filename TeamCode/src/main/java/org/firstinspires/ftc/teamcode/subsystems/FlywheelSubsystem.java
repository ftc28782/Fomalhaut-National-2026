package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Flywheel subsystem that controls the shooter motors and hood servo.
 */
public class FlywheelSubsystem extends SubsystemBase {

    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private final Servo hoodServo;

    // Motor constants
    public static final double TICKS_PER_REV = 28;

    // PIDF coefficients for shooter motors
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(270, 0, 0, 4);
    private double targetVelocityRPM = 3050;
    private double hoodPosition = 1;
    private boolean isRunning = false;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Configure motors
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set PIDF coefficients
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
    }

    @Override
    public void periodic() {
    }
    public void setVelocityForDistance(double distance) {
        targetVelocityRPM = 2991.758
                - 66.95706 * distance
                + 2.195716 * Math.pow(distance, 2)
                - 0.02787701 * Math.pow(distance, 3)
                + 0.0001660626 * Math.pow(distance, 4)
                - 3.770099e-7 * Math.pow(distance, 5);
        double ticksPerSecond = targetVelocityRPM * TICKS_PER_REV / 60;
        shooter1.setVelocity(ticksPerSecond);
        shooter2.setVelocity(ticksPerSecond);

        isRunning = targetVelocityRPM > 0;
    }

    /**
     * Calculate and set target velocity based on distance.
     * @param distance Distance to goal in inches
     */

    public void setHoodForDistance(double distance) {
        hoodPosition = 1.000169 + (-0.0006164855 - 1.000169) / (1 + Math.pow(distance / 49.03941, 21.62445));
        hoodServo.setPosition(hoodPosition);
    }
    /**
     * Start the flywheel at the current target velocity.
     */
    public void start() {
        isRunning = true;
        setVelocityForDistance(targetVelocityRPM);
    }

    /**
     * Stop the flywheel.
     */
    public void stop() {
        isRunning = false;
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    /**
     * Toggle the flywheel on/off.
     */
    public void toggle() {
        if (isRunning) {
            stop();
        } else {
            start();
        }
    }

    /**
     * Set the hood position.
     * @param position Position from 0 to 1
     */
    public void setHoodPosition(double position) {
        hoodPosition = position;
        hoodServo.setPosition(hoodPosition);
    }

    /**
     * Get the current velocity of shooter 2 in RPM.
     */
    public double getShooter2VelocityRPM() {
        return shooter2.getVelocity() * 60 / TICKS_PER_REV;
    }

    /**
     * Get the average current velocity in RPM.
     */
    public double getAverageVelocityRPM() {
        return getShooter2VelocityRPM();
    }

    /**
     * Get the velocity error (target - current) in RPM.
     */
    public double getVelocityError() {
        return targetVelocityRPM - getAverageVelocityRPM();
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(getVelocityError()) < 250;
    }

    /**
     * Check if flywheel is running.
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Get target velocity in RPM.
     */
    public double getTargetVelocityRPM() {
        return targetVelocityRPM;
    }

    /**
     * Get current hood position.
     */
    public double getHoodPosition() {
        return hoodPosition;
    }
}
