package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * Turret subsystem that controls the turret servos and tracks position via encoder.
 */
public class TurretSubsystem extends SubsystemBase {
    private final DcMotorEx encoderMotor, turretMotor; // Intake motor used for encoder reading

    private final PIDFController turretPID;

    // PID Constants
    public static double kP = 0.034;
    public static double kD = 0.0003;
    public static double kF = 0;

    // Encoder ticks to degrees conversion
    private static final double TICKS_TO_DEGREES = 100.35;

    private double currentAngle = 0;
    private double lastPower = 0;

    public TurretSubsystem(HardwareMap hardwareMap, DcMotorEx encoderMotor) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
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
//        if (currentAngle > 50 && power > 0) {
//            power = 0;
//        }
//        if (currentAngle < -50 && power < 0) {
//            power = 0;
//        }
        if (Math.abs(power) > 0.01) {
            if (power > 0) {
                power = power + kF;
            } else {
                power = power - kF;
            }
        }
        lastPower = power;
        turretMotor.setPower(power);
    }

    /**
     * Calculate PID output to track an angle error.
     * @param angleError The angle error in degrees
     * @return The calculated power
     */
    public double calculatePIDPower(double angleError) {
        if (Math.abs(angleError) < 1) {
            angleError = 0;
        }
        return turretPID.calculate(angleError);
    }

    public void GoalMovingOffset() {

        //Shot time = a equation based on distance being X and Shot time the Y --> shotTime = 321 - 42*distance - 0.02*Math.pow(distance,2);
        //xVelocity = robot.follower.getVelocity().getXComponent()
        //yVelocity = robot.follower.getVelocity().getYComponent()
        //xGoalOffset = robot.getGoalX - xVelocity * shotTime
        //yGoalOffset = robot.getGoalY - yVelocity * shotTime
    } // Com isso, vou alterar o cálculo do RPM do shooter e do hood do servo, porém os valores do offset não podem ser usados para substituir os valores normais.
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
