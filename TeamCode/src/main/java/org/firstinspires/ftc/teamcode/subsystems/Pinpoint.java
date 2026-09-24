package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.GoBildaPinpointDriver;

public class Pinpoint {

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D pose;
    private double yaw;
    private double x;
    private double y;

    //Todo calibrate

    public Pinpoint(GoBildaPinpointDriver pinpoint){
        this.pinpoint = pinpoint;
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    public void update(){
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.MM);
        y = pinpoint.getPosY(DistanceUnit.MM);
        yaw = pinpoint.getHeading(AngleUnit.RADIANS);
        pose = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, yaw);
    }

    public void setPosition(Pose2D pose){
        pinpoint.setPosition(pose);
    }

    public Pose2D getPosition(){
        return pose;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getYawInRadians(){
        return yaw;
    }

    public double getYawInDegrees(){
        return Math.toDegrees(yaw);
    }

}
