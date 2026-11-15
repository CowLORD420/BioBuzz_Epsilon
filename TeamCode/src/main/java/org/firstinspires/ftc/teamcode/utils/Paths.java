package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(86.4665, 73.5754, 180);
    private final Pose point2 = poseFactory.of(42.8814, 107.3285, -37.7548);

    public Path path = linePath(start, path1).linear(start, path1);

    public Path linePath(Pose start, Pose end){
        return com.pedropathing.api.Paths.line(start, end);
    }

    public Path curvePath(Pose start, Pose control, Pose end){
        return com.pedropathing.api.Paths.curve(start, control, end);
    }

    public Path compoundPath(Path... path){
        return com.pedropathing.api.Paths.path(path);
    }

}
