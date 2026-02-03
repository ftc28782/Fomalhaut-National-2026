package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command to run the flywheel with velocity based on distance.
 * Updates the target velocity continuously based on distance.
 */
public class FlywheelDistanceCommand extends CommandBase {

    private final FlywheelSubsystem flywheel;
    private final DoubleSupplier distanceSupplier;

    /**
     * Creates a new FlywheelDistanceCommand.
     *
     * @param flywheel The flywheel subsystem
     * @param distanceSupplier Supplier for the distance to goal
     */
    public FlywheelDistanceCommand(FlywheelSubsystem flywheel, DoubleSupplier distanceSupplier) {
        this.flywheel = flywheel;
        this.distanceSupplier = distanceSupplier;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.start();
    }

    @Override
    public void execute() {
        double distance = distanceSupplier.getAsDouble();
        flywheel.setVelocityForDistance(distance);
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
