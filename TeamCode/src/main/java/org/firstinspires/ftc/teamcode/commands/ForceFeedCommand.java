package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Command to force feed (run intake and transfer regardless of flywheel state).
 * Used for the "B" button functionality in the original code.
 */
public class ForceFeedCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;

    /**
     * Creates a new ForceFeedCommand.
     *
     * @param intake The intake subsystem
     * @param transfer The transfer subsystem
     */
    public ForceFeedCommand(IntakeSubsystem intake, TransferSubsystem transfer) {
        this.intake = intake;
        this.transfer = transfer;
        addRequirements(intake, transfer);
    }

    @Override
    public void initialize() {
        transfer.runForward();
    }

    @Override
    public void execute() {
        // Transfer continues running
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
