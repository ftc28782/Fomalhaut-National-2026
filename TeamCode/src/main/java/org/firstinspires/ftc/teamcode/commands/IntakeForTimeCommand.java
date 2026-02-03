package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command to run the intake for a specified duration.
 * Useful for autonomous intaking sequences.
 */
public class IntakeForTimeCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final double power;
    private final double timeoutSeconds;

    private long startTime;

    /**
     * Creates a new IntakeForTimeCommand.
     *
     * @param intake The intake subsystem
     * @param power Power to run the intake at (-1 to 1)
     * @param timeoutSeconds Duration to run for
     */
    public IntakeForTimeCommand(IntakeSubsystem intake, double power, double timeoutSeconds) {
        this.intake = intake;
        this.power = power;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(intake);
    }

    /**
     * Creates a new IntakeForTimeCommand that runs forward at full power.
     *
     * @param intake The intake subsystem
     * @param timeoutSeconds Duration to run for
     */
    public IntakeForTimeCommand(IntakeSubsystem intake, double timeoutSeconds) {
        this(intake, 1.0, timeoutSeconds);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        intake.setPower(power);
    }

    @Override
    public void execute() {
        // Intake runs at constant power
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        double elapsedSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
        return elapsedSeconds >= timeoutSeconds;
    }
}
