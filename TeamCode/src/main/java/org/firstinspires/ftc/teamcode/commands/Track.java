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
    private double error = 0;
    private double power = 0;

    //Todo don't forget to set the target poses for red and blue
    private Pose2D targetFieldPose;
    private static final Pose2D RED_TARGET_POSE = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private static final Pose2D BLUE_TARGET_POSE = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    public static double kF = 0.0;
    public static double kD = 0.0;
    public static double kI = 0.0;
    public static double kP = 0.0;

    //how close away to start
    public static double UNWIND_TRIGGER_DEG = 90;
    public static double UNWIND_KP = 0.02;
    public static double UNWIND_MAX_POWER = 0.6;
    //when to stop unwind
    public static double UNWIND_TOL_DEG = 3;

    //change these
    public static double MIN_DEG = 0;
    public static double MAX_DEG = 0;

    private enum TrackState{
        CAMERA,
        ODOMETRY,
        UNWIND
    }

    private TrackState trackState = TrackState.ODOMETRY;

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

        /*Todo change this into a state machine and make sure the changes from
           profiled unwind back to tarcking is smooth and somehow check if the error between what the
           camera error and odo error are somewhat close so it doesn't go crazy from a fake read
         */
        double odoErr = odoTargetAngle(pinpoint.getPosition(), targetFieldPose, turret.getYaw());

        if (trackState != TrackState.UNWIND && Math.abs(odoErr) > UNWIND_TRIGGER_DEG) {
            trackState = TrackState.UNWIND;
        }

        switch (trackState){
            case CAMERA:
                error = camera.getErrorInDegrees();
                power = pidf.calculate(error);
                if (!camera.hasTarget()){
                    pidf.reset();
                    trackState = TrackState.ODOMETRY;
                    break;
                }
                break;

            case ODOMETRY:
                error = pidf.calculate(odoErr);
                power = pidf.calculate(odoErr);
                if (camera.hasTarget()) {
                    pidf.reset();
                    trackState = TrackState.CAMERA;
                    break;
                }
                break;

            case UNWIND:
                power = Math.max(-UNWIND_MAX_POWER, Math.min(UNWIND_MAX_POWER, UNWIND_KP * odoErr));
                if (Math.abs(odoErr) < UNWIND_TOL_DEG) {
                    trackState = camera.hasTarget() ? TrackState.CAMERA : TrackState.ODOMETRY;
                    pidf.reset();
                }
                break;

        }
        if ((turret.atMaxTicks() && power > 0) || (turret.atMinTicks() && power < 0)) power = 0;


        turret.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        turret.setPower(0);
    }

    private double odoTargetAngle(Pose2D robotPose, Pose2D targetPose, double turretAngle) {
        double dx = targetPose.getX(DistanceUnit.MM) - robotPose.getX(DistanceUnit.MM);
        double dy = targetPose.getY(DistanceUnit.MM) - robotPose.getY(DistanceUnit.MM);

        double bearingToTarget = Math.atan2(dy, dx); // world-frame angle to target

        // angle the turret should point to, relative to chassis, normalized to [-180, 180]
        double targetAngle = bearingToTarget - robotPose.getHeading(AngleUnit.RADIANS);
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

        double yawDeg = Math.toDegrees(turretAngle);
        double desiredDeg = pickReachable(Math.toDegrees(targetAngle), yawDeg);

        // NOT wrapped: this error can exceed 180 when the turret must swing the long way
        return desiredDeg - yawDeg;
    }

    // Picks the equivalent angle (desired + k*360) inside the limits that is closest to the current yaw
    private double pickReachable(double desiredDeg, double currentDeg) {
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double c = desiredDeg + 360 * k;
            if (c < MIN_DEG || c > MAX_DEG) continue;
            if (Double.isNaN(best) || Math.abs(c - currentDeg) < Math.abs(best - currentDeg)) {
                best = c;
            }
        }
        // dead zone (range < 360): nothing reachable, so clamp to the nearest limit
        if (Double.isNaN(best)) {
            best = Math.max(MIN_DEG, Math.min(MAX_DEG, desiredDeg));
        }
        return best;
    }
}
