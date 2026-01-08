package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class TurretSubsystem extends SubsystemBase {
    public enum Alliance {
        BLUE, RED
    }

    private Alliance alliance = Alliance.BLUE;

    // TODO: Fill goal coordinates
    private final double GOAL_BLUE_X = 0.0, GOAL_BLUE_Y = 0.0;
    private final double GOAL_RED_X = 0.0, GOAL_RED_Y = 0.0;

    private final CRServo servoLeft, servoRight;
    private final DcMotorEx encoder;
    private final PIDFController controller;

    // Sensors
    private final Limelight3A limelightTurret;
    private final Limelight3A limelightChassis;
    private final GoBildaPinpointDriver pinpoint;

    // Aiming State
    public enum AimingMode {
        MANUAL,
        TURRET_CAM_TX,
        ROBOT_POSE
    }
    private AimingMode aimingMode = AimingMode.MANUAL;
    private Pose2D currentRobotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

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

        // Initialize Sensors
        limelightTurret = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelightChassis = hardwareMap.get(Limelight3A.class, "limelightChassis");

        limelightTurret.pipelineSwitch(0); // Assuming 0 is AprilTag
        limelightChassis.pipelineSwitch(0);

        limelightTurret.start();
        limelightChassis.start();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void periodic() {
        // Update Pose Estimation
        updatePoseEstimation();

        // Handle Aiming Modes
        switch (aimingMode) {
            case MANUAL:
                // Handled by manual() method calls
                break;
            case TURRET_CAM_TX:
                LLResult result = limelightTurret.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    // Simple alignment: turn based on TX
                    // Target is current pos + tx
                    setTargetPosition(getPositionDegrees() + tx);
                }
                break;
            case ROBOT_POSE:
                alignToTargetRobotPoseBased(currentRobotPose.getX(DistanceUnit.INCH), currentRobotPose.getY(DistanceUnit.INCH));
                break;
        }

        if (aimingMode != AimingMode.MANUAL) { // If automating, update PID
            double currentDegrees = getPositionDegrees();
            double power = controller.calculate(currentDegrees, targetDegrees);
             setPower(power);
        }
    }

    private void updatePoseEstimation() {
        pinpoint.update();
        Pose2D odoPose = pinpoint.getPosition(); // Current belief of position

        double fusedX = odoPose.getX(DistanceUnit.INCH);
        double fusedY = odoPose.getY(DistanceUnit.INCH);
        double fusedH = odoPose.getHeading(AngleUnit.DEGREES);

        // Simple Fusion: If Chassis Limelight sees a tag with high confidence, use it
        LLResult result = limelightChassis.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose();
            if (botPose3D != null) {
                // Limelight returns METERS (standard), convert to INCHES
                double visionX_in = botPose3D.getPosition().x * 39.3701;
                double visionY_in = botPose3D.getPosition().y * 39.3701;

                // Calculate distance between current odometry and vision to detect "jumps"
                double dist = Math.hypot(visionX_in - fusedX, visionY_in - fusedY);

                // ALPHA FILTER:
                // If the difference is huge (start of match or lost tracking), trust vision more (0.5).
                // If merely drifting, correct slowly (0.1).
                double alpha = (dist > 10.0) ? 0.5 : 0.1;

                fusedX = (1 - alpha) * fusedX + alpha * visionX_in;
                fusedY = (1 - alpha) * fusedY + alpha * visionY_in;

                // IMPORTANT: Write the corrected pose back to the Pinpoint hardware
                // This ensures that when vision is lost, odometry continues from the CORRECTED spot.
                Pose2D correctedPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, fusedH);
                pinpoint.setPosition(correctedPose);
            }
        }

        // Update our subsystem state
        currentRobotPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, fusedH);
    }

    public void setAimingMode(AimingMode mode) {
        this.aimingMode = mode;
        if (mode == AimingMode.MANUAL) {
            isManual = true;
        } else {
            isManual = false;
        }
    }

    public AimingMode getAimingMode() {
        return aimingMode;
    }

    public void manual(double power) {
        if (Math.abs(power) > 0.1) {
             setAimingMode(AimingMode.MANUAL);
             setPower(power);
        } else if (aimingMode == AimingMode.MANUAL) {
             setPower(0);
        }
    }

    public void setTargetPosition(double degrees) {
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
        double goalX = (alliance == Alliance.BLUE) ? GOAL_BLUE_X : GOAL_RED_X;
        double goalY = (alliance == Alliance.BLUE) ? GOAL_BLUE_Y : GOAL_RED_Y;

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        setTargetPosition(angleToGoal);
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public Pose2D getCurrentRobotPose() {
        return currentRobotPose;
    }

    public boolean isTargetVisible() {
        LLResult result = limelightTurret.getLatestResult();
        return result != null && result.isValid();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }
}
