package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pedro {
    private final Follower follower;

    public Pedro(Follower follower){
        this.follower = follower;
    }

    public void followPath(Path path){
        follower.follow(path);
    }

    public boolean isBusy(){
        return follower.following();
    }

    public void update(){
        follower.update();
    }

    public void setStartPose(Pose2D pose){
        follower.setPose(
                pose2DToPose(pose)
        );
    }

    private Pose pose2DToPose(Pose2D pose2D){
        return new Pose(
                pose2D.getY(DistanceUnit.MM) + 72,
                pose2D.getX(DistanceUnit.MM) - 72,
                pose2D.getHeading(AngleUnit.RADIANS) - Math.PI / 2
        );
    }

}
