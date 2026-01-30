package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.SunriseRobot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.concurrent.TimeUnit;

public class TurretSubsystem extends SubsystemBase {

    private boolean hasVision;
    Limelight3A limelightChassis;
    private double camX;
    private double camY;
    Follower follower;
    double alphaXY = 0.2;
    private double hood_position = 0.585;
    GamepadEx driver;
    SunriseRobot robot;
    CRServo servo1, servo2 = null;
    DcMotorEx intake = null;
    private double servoPower;
    Deadline IMUTimer;
    private double turretAngle;
    private double encoder;
    private double IMUDegress;
    PIDController turretPID;
    private IMU imu;
    private double c;
    public enum Alliance {
        BLUE, RED
    }
    private Alliance alliance = Alliance.BLUE;
    private final double GOAL_BLUE_X = -63, GOAL_BLUE_Y = -64;
    private final double GOAL_RED_X = -63, GOAL_RED_Y = 64;
    public enum AimingMode {
        MANUAL,
        TURRET_CAM_TX,
        ROBOT_POSE
    }
    private AimingMode aimingMode = AimingMode.MANUAL;

    // Encoder Gear: 50T, Turret Gear: 170T
    // Ratio: 170 / 50 = 3.4
    // REV Throughbore Encoder ticks: 8192
    private boolean isManual = true;
    private double targetDegrees = 0.0;
    private double fusedX;
    private double fusedY;

    public TurretSubsystem(HardwareMap hardwareMap) { //INIT

        //TURRET PID
        turretPID = new PIDController(0.014, 0.0, 0.0015);
        turretPID.setTolerance(0.5);
        turretPID.setSetPoint(0);

        //SERVOS
        servo1 = hardwareMap.get(CRServo.class, "turretLeft");
        servo2 = hardwareMap.get(CRServo.class, "rightTurret");

        //THROUGHBORE ENCODER
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //LIMELIGHT
        limelightChassis = hardwareMap.get(Limelight3A.class, "limelightTurret");
        limelightChassis.pipelineSwitch(0);
        limelightChassis.start();

        //PINPOINT
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0,0,0));

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        IMUTimer = new Deadline(500, TimeUnit.MILLISECONDS);
    }

    @Override
    public void periodic() {
        //THROUGHBORE ENCODER
        turretAngle = (intake.getCurrentPosition() / 77.369);

        //IMU RESET
        if (IMUTimer.hasExpired() && c < 1) {
            imu.resetYaw();
            IMUTimer.reset();
            c = c + 1;
        }
        IMUDegress = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //UPDATE FOLLOWER
        updatePoseEstimation();

        // Handle Aiming Modes
        switch (aimingMode) {
            case MANUAL:
                // Handled by manual() method calls
                break;
            case TURRET_CAM_TX:
                LLResult result = limelightChassis.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    setPower(tx * -0.1);
                }
                break;
            case ROBOT_POSE:
                alignToTargetRobotPoseBased();
                break;
        }

        if (aimingMode != AimingMode.MANUAL) {
             setPower(servoPower);
        }
    }

    private void updatePoseEstimation() {
        follower.update();

        // 1. Chassis Limelight
        hasVision = false;

        LLResult resultTurret = limelightChassis.getLatestResult();
        if (resultTurret != null && resultTurret.isValid()) {
            Pose3D camPose3D = resultTurret.getBotpose_MT2();
            if (camPose3D != null) {
                camX = camPose3D.getPosition().x * 39.3701;
                camY = camPose3D.getPosition().y * 39.3701;
                hasVision = true;
            }
        }

        // 2. Turret Limelight (Corrected)
//        LLResult resultTurret = limelightTurret.getLatestResult();
//        if (resultTurret != null && resultTurret.isValid()) {
//             Pose3D camPose3D = resultTurret.getBotpose();
//             if (camPose3D != null) {
//                 // Treating BotPose as Camera Field Pose (assuming 0 offset in LL Config)
//                 double camX = camPose3D.getPosition().x * 39.3701;
//                 double camY = camPose3D.getPosition().y * 39.3701;
//                 double camH = camPose3D.getOrientation().getYaw(AngleUnit.DEGREES);
//
//                 double turretRadius = 7.0;
//                 double robotX = camX - (turretRadius * Math.cos(Math.toRadians(camH)));
//                 double robotY = camY - (turretRadius * Math.sin(Math.toRadians(camH)));
//
//                 visionXSum += robotX;
//                 visionYSum += robotY;
//                 visionCount++;
//             }
//        }

        Pose followerPose = follower.getPose();
        double odoX = followerPose.getX();
        double odoY = followerPose.getY();

        fusedX = odoX;
        fusedY = odoY;

        double errorVision = Math.hypot(camX - odoX, camY - odoY);
        if (hasVision && errorVision > 0.5) {
            fusedX = odoX * (1 - alphaXY) + camX * alphaXY;
            fusedY = odoY * (1 - alphaXY) + camY * alphaXY;
        }

        // Update our subsystem state
        follower.setPose(new Pose(fusedX,fusedY,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
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
            this.servoPower = power;
            setPower(this.servoPower);
        } else if (aimingMode == AimingMode.MANUAL) {
            this.servoPower = 0;
            setPower(this.servoPower);
        }
    }

//    public double getPositionDegrees() {
//        return turretAngle = (intake.getCurrentPosition() / 77.369);
//    }

    private void setPower(double servoPower) {

        servoPower = Range.clip(servoPower, -1.0, 1.0);
        servo1.setPower(servoPower);
        servo2.setPower(servoPower);
    }

    public void alignToTargetRobotPoseBased() {
        double goalX = (alliance == Alliance.BLUE) ? GOAL_BLUE_X : GOAL_RED_X;
        double goalY = (alliance == Alliance.BLUE) ? GOAL_BLUE_Y : GOAL_RED_Y;

        double dx = goalX - fusedX;
        double dy = goalY - fusedY;

        double angleToGoal = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)) - IMUDegress - turretAngle);

        servoPower = -turretPID.calculate(angleToGoal);
                //+ (chassisTurn * 1);

        if (turretAngle > 135 && servoPower > 0) {
            servoPower = 0;
        }
        if (turretAngle < -135 && servoPower < 0) {
            servoPower = 0;
        }
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public Pose getCurrentRobotPose() {
        return follower.getPose();
    }

    public boolean isTargetVisible() {
        LLResult result = limelightChassis.getLatestResult();
        return result != null && result.isValid();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }
}
