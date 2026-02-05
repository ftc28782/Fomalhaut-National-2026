// java
package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Command to run the flywheel and dynamically update RPM based on Robot#getDistanceToGoal().
 * This version sempre se baseia em robot.getDistanceToGoal().
 */
public class FlywheelRunCommand extends CommandBase {

    private final FlywheelSubsystem flywheel;
    private final Robot robot;

    /**
     * @param flywheel The flywheel subsystem
     * @param robot The robot instance; used to call getDistanceToGoal()
     */
    public FlywheelRunCommand(FlywheelSubsystem flywheel, Robot robot) {
        this.flywheel = flywheel;
        this.robot = robot;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.start();
    }

    @Override
    public void execute() {
        // Sempre usa robot.getDistanceToGoal() em tempo de execução
        flywheel.setVelocityForDistance(robot.getDistanceToGoal());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            flywheel.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
