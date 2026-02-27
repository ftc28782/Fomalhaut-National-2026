package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Command to run the transfer servo.
 */
public class TransferRunCommand extends CommandBase {

    private final TransferSubsystem transfer;
    private final double power;

    /**
     * Creates a new TransferRunCommand.
     *
     * @param transfer The transfer subsystem
     * @param power Power to run the transfer at (-1 to 1)
     */
    public TransferRunCommand(TransferSubsystem transfer, double power) {
        this.transfer = transfer;
        this.power = power;
        addRequirements(transfer);
    }

    /**
     * Creates a new TransferRunCommand that runs forward at full power.
     *
     * @param transfer The transfer subsystem
     */
    public TransferRunCommand(TransferSubsystem transfer) {
        this(transfer, 1.0);
    }

    @Override
    public void initialize() {
        transfer.setPosition(power);
    }

    @Override
    public void execute() {
        // Transfer runs at constant power
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
