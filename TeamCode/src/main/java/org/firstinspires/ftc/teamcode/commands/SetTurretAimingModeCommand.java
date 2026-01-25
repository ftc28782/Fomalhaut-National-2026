package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class SetTurretAimingModeCommand extends InstantCommand {
    public SetTurretAimingModeCommand(TurretSubsystem turret, TurretSubsystem.AimingMode mode) {
        super(() -> {
            TurretSubsystem.AimingMode current = turret.getAimingMode();
            turret.setAimingMode(mode);
        }, turret);
    }
}

