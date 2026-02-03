package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command to run the intake.
 */
public class IntakeRunCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final double power;

    /**
     * Creates a new IntakeRunCommand.
     *
     * @param intake The intake subsystem
     * @param power Power to run the intake at (-1 to 1)
     */
    public IntakeRunCommand(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    /**
     * Creates a new IntakeRunCommand that runs forward at full power.
     *
     * @param intake The intake subsystem
     */
    public IntakeRunCommand(IntakeSubsystem intake) {
        this(intake, 1.0);
    }

    @Override
    public void initialize() {
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
        return false; // Runs until interrupted
    }
}
