package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.scheduler.Command;
import org.firstinspires.ftc.teamcode.subsystems.Pedro;

public class Follow implements Command {
    private final Pedro pedro;
    private final Path path;

    public Follow(Pedro pedro, Path path){
        this.pedro = pedro;
        this.path = path;
    }

    @Override
    public void start() {
        pedro.followPath(path);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return !pedro.isBusy();
    }

    @Override
    public void end() {

    }
}
