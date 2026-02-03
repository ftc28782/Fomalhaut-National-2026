package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

/**
 * Instant command to toggle the flywheel on/off.
 */
public class FlywheelToggleCommand extends InstantCommand {

    /**
     * Creates a new FlywheelToggleCommand.
     *
     * @param flywheel The flywheel subsystem
     */
    public FlywheelToggleCommand(FlywheelSubsystem flywheel) {
        super(flywheel::toggle, flywheel);
    }
}
