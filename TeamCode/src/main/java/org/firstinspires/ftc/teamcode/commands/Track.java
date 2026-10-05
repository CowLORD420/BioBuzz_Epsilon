package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.scheduler.Command;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PIDF;

//Todo change all the base 0 values
@Configurable
public class Track implements Command {
    private final Turret turret;
    private final Camera camera;
    private final Pinpoint pinpoint;
    private PIDF pidf = new PIDF(0,0,0,0);

    private Pose2D targetFieldPose;
    private static final Pose2D RED_TARGET_POSE = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private static final Pose2D BLUE_TARGET_POSE = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    public static double kF = 0.0;
    public static double kD = 0.0;
    public static double kI = 0.0;
    public static double kP = 0.0;

    public Track(Turret turret, Camera camera, Pinpoint pinpoint, Robot.Color color){
        this.turret = turret;
        this.camera = camera;
        this.pinpoint = pinpoint;
        switch (color){
            case RED:
                targetFieldPose = RED_TARGET_POSE;
                break;
            case BLUE:
                targetFieldPose = BLUE_TARGET_POSE;
                break;
        }
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

        double error;

        /*Todo change this into a state machine and make sure the changes from
           profiled unwind back to tarcking is smooth and somehow check if the error between what the
           camera error and odo error are somewhat close so it doesn't go crazy from a fake read
         */

        if (camera.hasTarget()) {
            error = camera.getErrorInDegrees();
        } else {
            error = Math.toDegrees(
                    odoTargetAngle(pinpoint.getPosition(), targetFieldPose, turret.getYaw())
            );
        }

        if (turret.atMaxTicks()) {
            // hand off to a profiled unwind move here instead of PID —
            return;
        }

        turret.setPower(pidf.calculate(error));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        turret.setPower(0);
    }

    private double odoTargetAngle(Pose2D robotPose, Pose2D targetPose, double turretAngle){
        double dx = targetPose.getX(DistanceUnit.MM) - robotPose.getX(DistanceUnit.MM);
        double dy = targetPose.getY(DistanceUnit.MM) - robotPose.getY(DistanceUnit.MM);

        double bearingToTarget = Math.atan2(dy, dx); // world-frame angle to target

        // angle turret should point to, relative to chassis
        double targetAngle = bearingToTarget - robotPose.getHeading(AngleUnit.RADIANS);
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle)); // normalize

        double error = targetAngle - turretAngle; // how much more to rotate from where it is now
        error = Math.atan2(Math.sin(error), Math.cos(error));

        return Math.toDegrees(error);
    }
}
