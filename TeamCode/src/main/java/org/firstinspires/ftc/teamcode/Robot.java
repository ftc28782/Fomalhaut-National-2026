package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * Robot class that contains all subsystems and shared hardware/sensors.
 */
public class Robot {

    public enum Alliance {
        BLUE, RED
    }

    // Subsystems
    public final TurretSubsystem turret;
    public final TransferSubsystem transfer;
    public final FlywheelSubsystem flywheel;
    public final IntakeSubsystem intake;

    // Shared Hardware
    public final Follower follower;
    public final IMU imu;
    public final Limelight3A limelight;

    // Goal coordinates
    public double goalX = -66;
    public double goalY = 66;

    // Vision and localization state
    private double camX = 0;
    private double camY = 0;
    // TeleOp drive offset (Heading adjustment for field-centric)
    private double driveOffset = -1.5708;
    private boolean hasVision = false;
    private double fusedX = 0;
    private double fusedY = 0;
    private double distanceToGoal = 0;
    private double angleToGoal = 0;

    // Alpha filter constant for vision fusion
    private double alphaXY = 0.08;

    // IMU initialization delay
    private final Deadline imuTimer;
    private int imuInitCount = 0;

    /**
     * Creates a new Robot with all subsystems.
     *
     * @param hardwareMap The hardware map from the OpMode
     */
    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap, intake.getMotor());
        transfer = new TransferSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        imuTimer = new Deadline(500, TimeUnit.MILLISECONDS);

        // Initialize Pinpoint/Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /**
     * Initialize with a specific starting pose.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param startPose The starting pose
     */
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        this(hardwareMap);
        follower.setPose(startPose);
    }

    /**
     * Update all robot state. Should be called each loop iteration.
     */
    public void update() {
        // Handle IMU reset delay
        if (imuTimer.hasExpired() && imuInitCount < 1) {
            imu.resetYaw();
            imuTimer.reset();
            imuInitCount++;
        }

        // Update subsystems
        turret.periodic();
        transfer.periodic();
        flywheel.periodic();
        intake.periodic();

        // Update follower
        follower.update();

        // Update limelight orientation
        limelight.updateRobotOrientation(getIMUYawDegrees());

        // Process vision
        updateVision();

        // Calculate distance and angle to goal
        calculateGoalMetrics();
    }

    /**
     * Update vision data from Limelight.
     */
    private void updateVision() {
        hasVision = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D camPose3D = result.getBotpose_MT2();
            if (camPose3D != null) {
                camX = camPose3D.getPosition().x * 39.3701; // Convert to inches
                camY = camPose3D.getPosition().y * 39.3701;
                hasVision = true;
            }
        }

        // Fuse vision with odometry
        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();

        fusedX = odoX;
        fusedY = odoY;

        double errorVision = Math.hypot(camX - odoX, camY - odoY);
        if (hasVision && errorVision > 3) {
            fusedX = odoX * (1 - alphaXY) + camX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + camY * alphaXY;
            follower.setPose(new Pose(fusedX, fusedY, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        }
    }

    /**
     * Calculate distance and angle to the goal.
     */
    private void calculateGoalMetrics() {
        double dx = goalX - fusedX;
        double dy = goalY - fusedY;

        distanceToGoal = Math.hypot(dx, dy);

        // Calculate angle to goal relative to turret
        double imuDegrees = getIMUYawDegrees();
        double turretAngle = turret.getCurrentAngle();
        angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - imuDegrees - turretAngle);
    }

    /**
     * Get IMU yaw in degrees.
     */
    public double getIMUYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Get IMU yaw in radians.
     */
    public double getIMUYawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Get the fused X position.
     */
    public double getFusedX() {
        return fusedX;
    }

    /**
     * Get the fused Y position.
     */
    public double getFusedY() {
        return fusedY;
    }

    /**
     * Get the camera X position from vision.
     */
    public double getCamX() {
        return camX;
    }

    /**
     * Get the camera Y position from vision.
     */
    public double getCamY() {
        return camY;
    }

    /**
     * Check if vision is available.
     */
    public boolean hasVision() {
        return hasVision;
    }

    /**
     * Get the distance to the goal.
     */
    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    /**
     * Get the angle to the goal (for turret tracking).
     */
    public double getAngleToGoal() {
        return angleToGoal;
    }

    /**
     * Set the goal coordinates.
     *
     * @param x Goal X coordinate
     * @param y Goal Y coordinate
     */
    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    /**
     * Sets the alliance to adjust drive orientation and goal position.
     *
     * @param alliance The alliance color
     */
    public void setAlliance(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            driveOffset = -1.5708;
            setGoal(-66, 66);
        } else {
            driveOffset = 1.5708;
            setGoal(-66, -66);
        }
    }

    /**
     * Set the alpha filter constant for vision fusion.
     *
     * @param alpha Alpha value (0-1)
     */
    public void setAlphaXY(double alpha) {
        this.alphaXY = alpha;
    }

    /**
     * Start teleop driving mode.
     */
    public void startTeleOpDrive() {
        follower.startTeleopDrive();
    }

    /**
     * Set teleop drive inputs.
     *
     * @param forward Forward/backward input
     * @param strafe Left/right input
     * @param rotate Rotation input
     */
    public void setTeleOpDrive(double forward, double strafe, double rotate) {
        follower.setTeleOpDrive(forward, strafe, rotate, false, driveOffset);
    }
}
