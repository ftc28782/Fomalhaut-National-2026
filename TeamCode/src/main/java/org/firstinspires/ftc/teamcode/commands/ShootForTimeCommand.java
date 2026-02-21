package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Command to shoot for a specified duration in autonomous.
 * Runs intake and transfer when flywheel is at speed, finishes after timeout.
 */
public class ShootForTimeCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final FlywheelSubsystem flywheel;
    private final double velocityTolerance;
    private final double timeoutSeconds;

    private long startTime;

    /**
     * Creates a new ShootForTimeCommand.
     *
     * @param intake The intake subsystem
     * @param transfer The transfer subsystem
     * @param flywheel The flywheel subsystem
     * @param timeoutSeconds Duration to shoot for
     * @param velocityTolerance Acceptable velocity error in RPM
     */
    public ShootForTimeCommand(IntakeSubsystem intake, TransferSubsystem transfer,
                               FlywheelSubsystem flywheel, double timeoutSeconds,
                               double velocityTolerance) {
        this.intake = intake;
        this.transfer = transfer;
        this.flywheel = flywheel;
        this.timeoutSeconds = timeoutSeconds;
        this.velocityTolerance = velocityTolerance;
        addRequirements(intake, transfer);
    }

    /**
     * Creates a new ShootForTimeCommand with default tolerance of 250 RPM.
     */
    public ShootForTimeCommand(IntakeSubsystem intake, TransferSubsystem transfer,
                               FlywheelSubsystem flywheel, double timeoutSeconds) {
        this(intake, transfer, flywheel, timeoutSeconds, 250);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
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
        double elapsedSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
        return elapsedSeconds >= timeoutSeconds;
    }
}
