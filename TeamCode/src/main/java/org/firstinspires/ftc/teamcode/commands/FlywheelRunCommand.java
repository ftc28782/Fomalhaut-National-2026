package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

/**
 * Command to run the flywheel at a specified velocity.
 */
public class FlywheelRunCommand extends CommandBase {

    private final FlywheelSubsystem flywheel;
    private final double velocityRPM;

    /**
     * Creates a new FlywheelRunCommand with a fixed velocity.
     *
     * @param flywheel The flywheel subsystem
     * @param velocityRPM Target velocity in RPM
     */
    public FlywheelRunCommand(FlywheelSubsystem flywheel, double velocityRPM) {
        this.flywheel = flywheel;
        this.velocityRPM = velocityRPM;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setVelocityRPM(velocityRPM);
    }

    @Override
    public void execute() {
        // Flywheel maintains velocity via motor PIDF
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
