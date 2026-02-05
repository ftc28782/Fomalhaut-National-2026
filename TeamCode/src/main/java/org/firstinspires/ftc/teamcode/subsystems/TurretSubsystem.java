package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * Turret subsystem that controls the turret servos and tracks position via encoder.
 */
public class TurretSubsystem extends SubsystemBase {

    private final CRServo turretLeft;
    private final CRServo turretRight;
    private final DcMotorEx encoderMotor; // Intake motor used for encoder reading

    private final PIDFController turretPID;

    // PID Constants
    public static double kP = 0.014;
    public static double kD = 0.0015;

    // Turret limits in degrees
    public static final double MAX_ANGLE = 115;
    public static final double MIN_ANGLE = -115;

    // Encoder ticks to degrees conversion
    private static final double TICKS_TO_DEGREES = 77.369;

    private double currentAngle = 0;
    private double lastPower = 0;

    public TurretSubsystem(HardwareMap hardwareMap, DcMotorEx encoderMotor) {
        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "rightTurret");
        this.encoderMotor = encoderMotor;

        turretPID = new PIDFController(kP, 0.0, kD, 0);
        turretPID.setTolerance(0.5);
        turretPID.setSetPoint(0);
    }

    @Override
    public void periodic() {
        // Update current angle from encoder
        currentAngle = (encoderMotor.getCurrentPosition() / TICKS_TO_DEGREES);
    }

    /**
     * Set raw power to the turret servos.
     * @param power Power to set (-1 to 1)
     */
    public void setPower(double power) {
        // Apply limits
        if (currentAngle > MAX_ANGLE && power > 0) {
            power = 0;
        }
        if (currentAngle < MIN_ANGLE && power < 0) {
            power = 0;
        }

        lastPower = power;
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    /**
     * Calculate PID output to track an angle error.
     * @param angleError The angle error in degrees
     * @param chassisTurnCompensation Compensation for chassis rotation
     * @return The calculated power
     */
    public double calculatePIDPower(double angleError, double chassisTurnCompensation) {
        if (Math.abs(angleError) < 0.4) {
            angleError = 0;
        }
        return -turretPID.calculate(angleError) + (chassisTurnCompensation * 1);
    }

    /**
     * Get the current turret angle in degrees.
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    /**
     * Update PID coefficients.
     */
    public void setPIDCoefficients(double p, double d) {
        kP = p;
        kD = d;
        turretPID.setPIDF(p, 0, d, 0);
    }

    /**
     * Stop the turret.
     */
    public void stop() {
        setPower(0);
    }

    public double getLastPower() {
        return lastPower;
    }
}
