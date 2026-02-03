package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

/**
 * Command to follow a Pedro Pathing path chain.
 * Finishes when the path is complete.
 */
public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain path;

    /**
     * Creates a new FollowPathCommand.
     *
     * @param follower The Pedro Pathing follower
     * @param path The path chain to follow
     */
    public FollowPathCommand(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
        // Note: Follower is not a subsystem, so no requirements needed
    }

    @Override
    public void initialize() {
        follower.followPath(path);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public void end(boolean interrupted) {
        // Path following complete or interrupted
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
