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
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(7, 0, 0, 14.2);

    // Target velocity calculation constants (based on distance)
    private static final double VELOCITY_A = 1364.45;
    private static final double VELOCITY_B = 21.20203;
    private static final double VELOCITY_C = -0.03188598;

    // Hood position calculation constants (commented in original)
    // private static final double HOOD_A = -0.8909748;
    // private static final double HOOD_B = 0.0521592;
    // private static final double HOOD_C = -0.0006677889;
    // private static final double HOOD_D = 0.000003639036;
    // private static final double HOOD_E = -7.141361e-9;

    private double targetVelocityRPM = 3500;
    private double hoodPosition = 1;
    private boolean isRunning = false;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Configure motors
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set PIDF coefficients
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
    }

    @Override
    public void periodic() {
        // Update hood position
        hoodServo.setPosition(hoodPosition);
    }

    /**
     * Set the target velocity in RPM and run the flywheel.
     * @param rpm Target velocity in RPM
     */
    public void setVelocityRPM(double rpm) {
        targetVelocityRPM = rpm;
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        shooter1.setVelocity(ticksPerSecond);
        shooter2.setVelocity(ticksPerSecond);
        isRunning = rpm > 0;
    }

    /**
     * Calculate and set target velocity based on distance.
     * @param distance Distance to goal in inches
     */
    public void setVelocityForDistance(double distance) {
        double rpm = VELOCITY_A + VELOCITY_B * distance + VELOCITY_C * Math.pow(distance, 2);
        targetVelocityRPM = rpm;
        if (isRunning) {
            setVelocityRPM(rpm);
        }
    }

    /**
     * Calculate target velocity for a given distance without setting it.
     * @param distance Distance to goal in inches
     * @return Calculated velocity in RPM
     */
    public double calculateVelocityForDistance(double distance) {
        return VELOCITY_A + VELOCITY_B * distance + VELOCITY_C * Math.pow(distance, 2);
    }

    /**
     * Start the flywheel at the current target velocity.
     */
    public void start() {
        isRunning = true;
        setVelocityRPM(targetVelocityRPM);
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
    }

    /**
     * Calculate and set hood position based on distance.
     * @param distance Distance to goal in inches
     */
    public void setHoodForDistance(double distance) {
        // Formula from original code (commented out)
        // hoodPosition = HOOD_A + HOOD_B * distance + HOOD_C * Math.pow(distance, 2)
        //              + HOOD_D * Math.pow(distance, 3) + HOOD_E * Math.pow(distance, 4);
        // Currently using fixed position as in original
    }

    /**
     * Get the current velocity of shooter 1 in RPM.
     */
    public double getShooter1VelocityRPM() {
        return shooter1.getVelocity() * 60 / TICKS_PER_REV;
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
        return (getShooter1VelocityRPM() + getShooter2VelocityRPM()) / 2;
    }

    /**
     * Get the velocity error (target - current) in RPM.
     */
    public double getVelocityError() {
        return targetVelocityRPM - getAverageVelocityRPM();
    }

    /**
     * Check if flywheel is at target velocity (within tolerance).
     * @param tolerance Acceptable error in RPM
     */
    public boolean isAtTargetVelocity(double tolerance) {
        return Math.abs(getVelocityError()) < tolerance;
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
