package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Command to shoot when flywheel is at speed.
 * Runs intake and transfer when flywheel velocity error is within tolerance.
 */
public class ShootCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final FlywheelSubsystem flywheel;
    private final double velocityTolerance;

    /**
     * Creates a new ShootCommand.
     *
     * @param intake The intake subsystem
     * @param transfer The transfer subsystem
     * @param flywheel The flywheel subsystem
     * @param velocityTolerance Acceptable velocity error in RPM
     */
    public ShootCommand(IntakeSubsystem intake, TransferSubsystem transfer,
                        FlywheelSubsystem flywheel, double velocityTolerance) {
        this.intake = intake;
        this.transfer = transfer;
        this.flywheel = flywheel;
        this.velocityTolerance = velocityTolerance;
        addRequirements(intake, transfer);
    }

    /**
     * Creates a new ShootCommand with default tolerance of 250 RPM.
     */
    public ShootCommand(IntakeSubsystem intake, TransferSubsystem transfer,
                        FlywheelSubsystem flywheel) {
        this(intake, transfer, flywheel, 285);
    }

    @Override
    public void execute() {
        if (flywheel.isAtTargetVelocity()) {
            intake.runForward();
            transfer.runForward();
        } else {
            intake.stop();
            transfer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
