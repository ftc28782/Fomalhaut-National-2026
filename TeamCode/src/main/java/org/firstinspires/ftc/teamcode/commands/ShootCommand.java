package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class ShootCommand extends CommandBase {
    private final FlywheelSubsystem flywheel;

    public ShootCommand(FlywheelSubsystem flywheel) {
        this.flywheel = flywheel;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        flywheel.setShooterPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setShooterPower(0);
    }
}

