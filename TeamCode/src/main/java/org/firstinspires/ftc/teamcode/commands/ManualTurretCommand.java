package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;

public class ManualTurretCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final DoubleSupplier rotationSupplier;

    public ManualTurretCommand(TurretSubsystem turret, DoubleSupplier rotationSupplier) {
        this.turret = turret;
        this.rotationSupplier = rotationSupplier;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.manual(rotationSupplier.getAsDouble());
    }
}

