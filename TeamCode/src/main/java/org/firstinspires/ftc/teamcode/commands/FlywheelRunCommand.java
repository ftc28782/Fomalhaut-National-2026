package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command to run the flywheel and dynamically update RPM based on distance.
 */
public class FlywheelRunCommand extends CommandBase {

    private final FlywheelSubsystem flywheel;
    private final DoubleSupplier distanceSupplier;

    /**
     * @param flywheel The flywheel subsystem
     * @param distanceSupplier Lambda to get the current distance to goal
     */
    public FlywheelRunCommand(FlywheelSubsystem flywheel, DoubleSupplier distanceSupplier) {
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
        // Atualiza o RPM conforme a distância muda
        flywheel.setVelocityForDistance(distanceSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Se quisermos que ele continue girando mesmo após o comando acabar
        // (ex: durante a transição de um caminho para o chute), 
        // comentamos o flywheel.stop() ou tratamos o interrupted.
        if (!interrupted) {
            flywheel.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
