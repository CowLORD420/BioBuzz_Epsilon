package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.scheduler.Command;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.opencv.core.Mat;

@Configurable
public class Track implements Command {
    private final Turret turret;
    private final Camera camera;
    private final Pinpoint pinpoint;
    private PIDF pidf = new PIDF(0,0,0,0);

    public static double kF = 0.0;
    public static double kD = 0.0;
    public static double kI = 0.0;
    public static double kP = 0.0;

    public Track(Turret turret, Camera camera, Pinpoint pinpoint){
        this.turret = turret;
        this.camera = camera;
        this.pinpoint = pinpoint;
    }

    @Override
    public void start() {
        pidf.setSetPoint(0);
        pidf.setTolerance(1);
        pidf.setOutputLimits(-1, 1);
    }

    @Override
    public void update() {
        pidf.setPIDF(kP, kI, kD, kF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {

    }

    private double odoTargetAngle(Pose2D robotPose, Pose2D targetPose, double turretAngle){
        double dx = targetPose.getX(DistanceUnit.MM) - robotPose.getX(DistanceUnit.MM);
        double dy = targetPose.getY(DistanceUnit.MM) - robotPose.getY(DistanceUnit.MM);

        double bearingToTarget = Math.atan2(dy, dx);              // world-frame angle to target
        double targetAngle = bearingToTarget - robotPose.getHeading(AngleUnit.RADIANS); // angle turret should point to, relative to chassis
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle)); // normalize

        double error = targetAngle - turretAngle; // how much more to rotate from where it is now
        error = Math.atan2(Math.sin(error), Math.cos(error));

        return error;
    }
}
