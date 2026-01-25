package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakePullCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakePullCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakePower(1);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakePower(0);
    }
}

