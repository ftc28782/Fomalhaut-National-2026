package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command to track a goal angle with the turret using PID control.
 * This command runs continuously and is typically the default command.
 */
public class TurretTrackCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final DoubleSupplier angleErrorSupplier;
    private final DoubleSupplier chassisTurnSupplier;

    /**
     * Creates a new TurretTrackCommand.
     *
     * @param turret The turret subsystem
     * @param angleErrorSupplier Supplier for the angle error to track
     * @param chassisTurnSupplier Supplier for chassis turn compensation (e.g., from gamepad)
     */
    public TurretTrackCommand(TurretSubsystem turret,
                               DoubleSupplier angleErrorSupplier,
                               DoubleSupplier chassisTurnSupplier) {
        this.turret = turret;
        this.angleErrorSupplier = angleErrorSupplier;
        this.chassisTurnSupplier = chassisTurnSupplier;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double angleError = angleErrorSupplier.getAsDouble();
        double chassisTurn = chassisTurnSupplier.getAsDouble();
        double power = turret.calculatePIDPower(angleError, chassisTurn);
        turret.setPower(power);
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
