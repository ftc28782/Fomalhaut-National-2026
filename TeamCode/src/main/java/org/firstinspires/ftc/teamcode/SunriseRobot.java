package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class SunriseRobot extends Robot {

    public TurretSubsystem turretSubsystem;
    public HardwareMap hwMap;

    public enum OpModeType {
        TELEOP, AUTO;
    }

    // the constructor with a specified opmode type
    public SunriseRobot(OpModeType type, HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;
//        turretSubsystem = new TurretSubsystem(hwMap);
        if (type == OpModeType.TELEOP) {
            initTeleOp();
        } else {
            initAuto();
        }
    }

    private void initTeleOp() {
        // initialize teleop-specific scheduler
    }

    private void initAuto() {
        // initialize auto-specific scheduler
    }

}
