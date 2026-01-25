package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ToggleTurretAimingModeCommand extends InstantCommand {
    public ToggleTurretAimingModeCommand(TurretSubsystem turret) {
        super(() -> {
            TurretSubsystem.AimingMode current = turret.getAimingMode();
            if (current == TurretSubsystem.AimingMode.MANUAL) {
                turret.setAimingMode(TurretSubsystem.AimingMode.TURRET_CAM_TX);
            } else if (current == TurretSubsystem.AimingMode.TURRET_CAM_TX) {
                turret.setAimingMode(TurretSubsystem.AimingMode.ROBOT_POSE);
            } else {
                turret.setAimingMode(TurretSubsystem.AimingMode.MANUAL);
            }
        }, turret);
    }
}

