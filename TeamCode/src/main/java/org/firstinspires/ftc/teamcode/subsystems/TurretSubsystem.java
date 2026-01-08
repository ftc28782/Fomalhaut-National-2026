package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class TurretSubsystem extends SubsystemBase {
    // Goal position in arena
    private final double GOAL_X = 0.0, GOAL_Y = 0.0;

    private final CRServo servoLeft, servoRight;
    private final DcMotorEx encoder;
    private final PIDFController controller;

    // Encoder Gear: 50T, Turret Gear: 170T
    // Ratio: 170 / 50 = 3.4
    // REV Throughbore Encoder ticks: 8192
    private static final double TICKS_PER_REV = 8192.0;
    private static final double GEAR_RATIO = 3.4;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // TODO: Tune PIDF values
    public static double kV = 0.15;
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.05;

    private boolean isManual = true;
    private double targetDegrees = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        // Initialize servos
        servoLeft = hardwareMap.get(CRServo.class, "turretLeft");
        servoRight = hardwareMap.get(CRServo.class, "turretRight");

        servoRight.setDirection(CRServo.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(kP, kI, kD, kV);
    }

    @Override
    public void periodic() {
        if (!isManual) {
            double currentDegrees = getPositionDegrees();
            double power = controller.calculate(currentDegrees, targetDegrees);

            // Optional: Feedforward could be added here if needed

            setPower(power);
        }
    }

    public void manual(double power) {
        isManual = true;
        setPower(power);
    }

    public void setTargetPosition(double degrees) {
        isManual = false;
        this.targetDegrees = Range.clip(degrees, 0, 355);
//      this.targetDegrees = degrees;
        controller.setSetPoint(targetDegrees);
    }

    public double getPositionDegrees() {
        return encoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    private void setPower(double power) {
        if (getPositionDegrees() > 355 && power > 0) power = 0;
        if (getPositionDegrees() < 0 && power < 0) power = 0;

        power = Range.clip(power, -1.0, 1.0);
        servoLeft.setPower(power);
        servoRight.setPower(power);
    }

    public void updatePID(double v, double p, double i, double d) {
        controller.setPIDF(p, i, d, v);
    }

    public void alignToTargetTXbased(double tx) {
        setTargetPosition(getPositionDegrees() + tx);
    }


    public void alignToTargetRobotPoseBased(double robotX, double robotY) {
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        setTargetPosition(angleToGoal);
    }
}
