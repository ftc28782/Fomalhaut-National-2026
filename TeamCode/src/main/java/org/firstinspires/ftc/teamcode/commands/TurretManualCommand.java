package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command to manually control the turret with a power supplier.
 */
public class TurretManualCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final DoubleSupplier powerSupplier;

    /**
     * Creates a new TurretManualCommand.
     *
     * @param turret The turret subsystem
     * @param powerSupplier Supplier for the manual power input
     */
    public TurretManualCommand(TurretSubsystem turret, DoubleSupplier powerSupplier) {
        this.turret = turret;
        this.powerSupplier = powerSupplier;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
