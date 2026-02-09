package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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


    // Subsystems
    public final TurretSubsystem turret;
    public final TransferSubsystem transfer;
    public final FlywheelSubsystem flywheel;
    public final IntakeSubsystem intake;

    // Shared Hardware
    public final Follower follower;
    public final Limelight3A limelight;

    // Goal coordinates
    public double goalX;
    public double goalY;

    // Vision and localization state
    private double camX = 0;
    private double camY = 0;
    // TeleOp drive offset (Heading adjustment for field-centric)
    private double driveOffset = -1.5708;
    private boolean hasVision = false;
    private double distanceToGoal = 0;
    private double angleToGoal = 0;

    // Alpha filter constant for vision fusion
    private double alphaXY = 0.08;
    private double imuDegrees;
    Pose followerPose;
    public enum Alliance {
        AUTO_BLUE,
        AUTO_RED,
        RED,
        BLUE
    }
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

        // Update subsystems
        turret.periodic();
        transfer.periodic();
        flywheel.periodic();
        intake.periodic();

        // Update follower
        follower.update();
        follower.getPose().getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        // Update limelight orientation
        limelight.updateRobotOrientation(getHeadingDegrees());

        // Calculate distance and angle to goal
        calculateGoalMetrics();
    }

    /**
     * Update vision data from Limelight.
     */
    public void updateVision() {

        hasVision = false;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D camPose3D = result.getBotpose_MT2();
            if (camPose3D != null) {
                camX = (camPose3D.getPosition().x * 39.3701);
                camY = (camPose3D.getPosition().y * 39.3701);
                    double errorVision = Math.hypot(camX - follower.getPose().getX(), camY - follower.getPose().getY());
                if (errorVision > 3) {
                    follower.setPose(new Pose(camX,camY,follower.getHeading()));
                    hasVision = true;
                }
            }
        }
    }

    /**
     * Calculate distance and angle to the goal.
     */
    private void calculateGoalMetrics() {
        double dx = goalX - follower.getPose().getX();
        double dy = goalY - follower.getPose().getY();

        distanceToGoal = Math.hypot(dx, dy);

        // Calculate angle to goal relative to turret
        imuDegrees = normalizeDegrees(getHeadingDegrees());
        double turretAngle = turret.getCurrentAngle();
        angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - imuDegrees - turretAngle);
    }
        public double FixedIMUDegrees() {
        return imuDegrees;
        }

    /**
     * Sets the alliance to adjust drive orientation and goal position.
     *
     * @param alliance The alliance color
     */
    public void setAlliance(Alliance alliance) {
        if (alliance == Alliance.BLUE){
            setGoal(-66, -66);
            driveOffset = -1.5707963267948966;
        } else if (alliance == Alliance.RED) {
            setGoal(-66, 66);
            driveOffset = 1.5707963267948966;
        } else if (alliance == Alliance.AUTO_BLUE) {
            setGoal(8, 136);
        } else if (alliance == Alliance.AUTO_RED){
            setGoal(136, 136);
        }
    }
    /**
     * Normaliza um ângulo em graus para o intervalo [-180, 180).
     */
    public static double normalizeDegrees(double degrees) {
        degrees %= 360.0;
        if (degrees >= 180.0) degrees -= 360.0;
        if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
    /**
     * Get IMU yaw in degrees.
     */
    public double getHeading() {
        return follower.getHeading();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(follower.getHeading());
    }

    /**
     * Get the X position.
     */
    public double getX() {
        return follower.getPose().getX();
    }

    /**
     * Get the Y position.
     */
    public double getY() {
        return follower.getPose().getY();
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

    public double getGoalX() {
        return goalX;
    }

    public double getGoalY() {
        return goalY;
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
