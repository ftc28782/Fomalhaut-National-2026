package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
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
    private final double GOAL_BLUE_X = -63, GOAL_BLUE_Y = -64;
    private final double GOAL_RED_X = -63, GOAL_RED_Y = 64;

//    private final CRServo servo1/*, servoRight*/;
//    private final DcMotorEx encoder;
    private final PIDFController controller;

    // Sensors
    private final Limelight3A limelightTurret;
//    private final Limelight3A limelightChassis;
//    private final GoBildaPinpointDriver pinpoint;

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
    public static double kF = 0.15;
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.05;

    private boolean isManual = true;
    private double targetDegrees = 0.0;


    public TurretSubsystem(HardwareMap hardwareMap) {
        // Initialize servos
//        servo1 = hardwareMap.get(CRServo.class, "turretLeft");
//        servoRight = hardwareMap.get(CRServo.class, "turretRight");

//        servoRight.setDirection(CRServo.Direction.REVERSE);

//        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
//        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // the turret encoder should increase when turning left
        // encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(kP, kI, kD, kF);

        // Initialize Sensors
        limelightTurret = hardwareMap.get(Limelight3A.class, "limelightTurret");
//        limelightChassis = hardwareMap.get(Limelight3A.class, "limelightChassis");

        limelightTurret.pipelineSwitch(0); // Assuming 0 is AprilTag
//        limelightChassis.pipelineSwitch(0);

        limelightTurret.start();
//        limelightChassis.start();

//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        pinpoint.resetPosAndIMU();
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
                    setPower(tx * -0.1);
                }
                break;
            case ROBOT_POSE:
                alignToTargetRobotPoseBased(currentRobotPose.getX(DistanceUnit.INCH), currentRobotPose.getY(DistanceUnit.INCH));
                break;
        }

        if (aimingMode != AimingMode.MANUAL) { // If automating, update PID
//            double currentDegrees = getPositionDegrees();
//            double power = controller.calculate(currentDegrees, targetDegrees);
//             setPower(power);
        }
    }

    private void updatePoseEstimation() {
//        pinpoint.update();
//        Pose2D odoPose = pinpoint.getPosition(); // Current belief of position

//        double fusedX = odoPose.getX(DistanceUnit.INCH);
//        double fusedY = odoPose.getY(DistanceUnit.INCH);
//        double fusedH = odoPose.getHeading(AngleUnit.DEGREES);

        double visionXSum = 0;
        double visionYSum = 0;
        int visionCount = 0;

        // 1. Chassis Limelight
//        LLResult resultChassis = limelightChassis.getLatestResult();
//        if (resultChassis != null && resultChassis.isValid()) {
//             Pose3D botPose3D = resultChassis.getBotpose();
//             if (botPose3D != null) {
//                 visionXSum += botPose3D.getPosition().x * 39.3701; // Meters to Inches
//                 visionYSum += botPose3D.getPosition().y * 39.3701;
//                 visionCount++;
//             }
//        }

        // 2. Turret Limelight (Corrected)
        LLResult resultTurret = limelightTurret.getLatestResult();
        if (resultTurret != null && resultTurret.isValid()) {
             Pose3D camPose3D = resultTurret.getBotpose();
             if (camPose3D != null) {
                 // Treating BotPose as Camera Field Pose (assuming 0 offset in LL Config)
                 double camX = camPose3D.getPosition().x * 39.3701;
                 double camY = camPose3D.getPosition().y * 39.3701;
                 double camH = camPose3D.getOrientation().getYaw(AngleUnit.DEGREES);

                 // Correct for Turret Camera Offset (Back-calculate Robot Center)
                 // TODO: Measure exact radius from turret center to camera lens
                 double turretRadius = 7.0; // Estimate: 6 inches

                 // Calculate Robot Center based on Camera Field Pose and Camera Field Heading
                 // We assume camera is mounted facing 'forward' on the turret
                 double robotX = camX - (turretRadius * Math.cos(Math.toRadians(camH)));
                 double robotY = camY - (turretRadius * Math.sin(Math.toRadians(camH)));

                 visionXSum += robotX;
                 visionYSum += robotY;
                 visionCount++;
             }
        }

        // Fusion Logic
        if (visionCount > 0) {
            double visionX = visionXSum / visionCount;
            double visionY = visionYSum / visionCount;

            // Calculate distance between current odometry and vision to detect "jumps"
//            double dist = Math.hypot(visionX - fusedX, visionY - fusedY);

            // ALPHA FILTER:
            // If the difference is huge (start of match or lost tracking), trust vision more.
            // If merely drifting, correct slowly.
//            double alpha = (dist > 10.0) ? 0.5 : 0.05 * visionCount; // More sensors = slightly more trust

//            fusedX = (1 - alpha) * fusedX + alpha * visionX;
//            fusedY = (1 - alpha) * fusedY + alpha * visionY;

            // IMPORTANT: Write the corrected pose back to the Pinpoint hardware
            // This ensures that when vision is lost, odometry continues from the CORRECTED spot.
//            Pose2D correctedPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, fusedH);
//            pinpoint.setPosition(correctedPose);
        }

        // Update our subsystem state
//        currentRobotPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, fusedH);
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

//    public double getPositionDegrees() {
//        return encoder.getCurrentPosition() / TICKS_PER_DEGREE;
//    }

    private static final double MAX_DEGREES = 177.0;
    private static final double MIN_DEGREES = -177.0;

    private void setPower(double power) {
//        double currentPos = getPositionDegrees();
        // Soft Stops
//        if (currentPos > MAX_DEGREES && power > 0) power = 0;
//        if (currentPos < MIN_DEGREES && power < 0) power = 0;

        power = Range.clip(power, -1.0, 1.0);
//        servo1.setPower(power);
//        servoRight.setPower(power);
    }

    public void setTargetPosition(double degrees) {
        this.targetDegrees = Range.clip(degrees, MIN_DEGREES, MAX_DEGREES);
        controller.setSetPoint(targetDegrees);
    }

    public void alignToTargetRobotPoseBased(double robotX, double robotY) {
        double goalX = (alliance == Alliance.BLUE) ? GOAL_BLUE_X : GOAL_RED_X;
        double goalY = (alliance == Alliance.BLUE) ? GOAL_BLUE_Y : GOAL_RED_Y;

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Calculate Absolute Field Angle to Goal
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Calculate Robot Heading (need to fetch from Pinpoint or Odometry)
        // Note: currentRobotPose is updated in periodic() from Pinpoint+Vision
        double robotHeading = currentRobotPose.getHeading(AngleUnit.DEGREES);

        // Calculate Turret Relative Angle (Goal - Robot)
        double relativeAngle = fieldAngleToGoal - robotHeading;

        // Normalize to -180 to 180 to find shortest path
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle <= -180) relativeAngle += 360;

        setTargetPosition(relativeAngle);
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
