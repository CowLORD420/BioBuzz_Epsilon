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
    private final PIDF pidf = new PIDF(0, 0, 0, 0);

    //Todo don't forget to set the target poses for red and blue
    private final Pose2D targetFieldPose;
    private static final Pose2D RED_TARGET_POSE  = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private static final Pose2D BLUE_TARGET_POSE = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    // TUNE: kF -> kP -> kD, kI last. Check sign: +power should reduce +error
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    //how close away to start
    // TUNE: odoErr (deg) that triggers unwind
    public static double UNWIND_ENTER_DEG = 90;
    //when to stop unwind
    // TUNE: hand off to PIDF here, keep well below ENTER
    public static double UNWIND_EXIT_DEG = 20;
    // TUNE: unwind speed vs overshoot
    public static double UNWIND_KP = 0.02;
    // TUNE: lower if it slams limits or yanks cables
    public static double UNWIND_MAX_POWER = 0.6;
    // TUNE: smallest power that overcomes static friction
    public static double UNWIND_MIN_POWER = 0.08;

    // TUNE: max |camErr - odoErr| to trust the camera
    public static double CAMERA_GATE_DEG = 15;
    // TUNE: good frames needed to lock on
    public static int CAMERA_ACQUIRE_LOOPS = 3;
    // TUNE: bad frames before falling back to odo
    public static int CAMERA_LOST_LOOPS = 5;

    // TUNE: max power change per sec (smooths handoffs)
    public static double MAX_POWER_SLEW_PER_SEC = 4.0;

    //change these
    // TUNE: hard-stop yaw in deg, pulled in a few deg for margin
    public static double MIN_DEG = 0;
    public static double MAX_DEG = 0;

    private enum TrackState { CAMERA, ODOMETRY, UNWIND }
    private TrackState trackState = TrackState.ODOMETRY;

    private double power = 0;
    private double lastTime = 0;
    private int goodCount = 0;
    private int badCount = 0;

    public Track(Turret turret, Camera camera, Pinpoint pinpoint, Robot.Color color) {
        this.turret = turret;
        this.camera = camera;
        this.pinpoint = pinpoint;
        this.targetFieldPose = (color == Robot.Color.RED) ? RED_TARGET_POSE : BLUE_TARGET_POSE;
    }

    @Override
    public void start() {
        pidf.setSetPoint(0);
        pidf.setTolerance(5);          // TUNE: deg counted as on target
        pidf.setOutputLimits(-1, 1);
        pidf.reset();
        trackState = TrackState.ODOMETRY;
        power = 0;
        lastTime = 0;
        goodCount = badCount = 0;
    }

    @Override
    public void update() {
        pidf.setPIDF(kP, kI, kD, kF);

        double now = System.nanoTime() / 1e9;
        double dt = (lastTime == 0) ? 0.02 : Math.min(now - lastTime, 0.1);
        lastTime = now;

        double odoErr = odoTargetAngle(pinpoint.getPosition(), targetFieldPose, turret.getYaw());

        // camera and odo errors should agree, if not the camera read is probably fake
        boolean hasTarget = camera.hasTarget();
        double camErr = hasTarget ? camera.getErrorInDegrees() : 0;
        boolean plausible = hasTarget && Math.abs(camErr - odoErr) < CAMERA_GATE_DEG;

        if (plausible) { goodCount++; badCount = 0; }
        else           { badCount++;  goodCount = 0; }

        // unwind only starts from ODOMETRY so odo drift can't pull us off a camera lock
        switch (trackState) {
            case UNWIND:
                if (Math.abs(odoErr) < UNWIND_EXIT_DEG) {
                    pidf.reset();
                    trackState = (goodCount >= CAMERA_ACQUIRE_LOOPS) ? TrackState.CAMERA : TrackState.ODOMETRY;
                }
                break;

            case ODOMETRY:
                if (Math.abs(odoErr) > UNWIND_ENTER_DEG) {
                    trackState = TrackState.UNWIND;
                } else if (goodCount >= CAMERA_ACQUIRE_LOOPS) {
                    pidf.reset();
                    trackState = TrackState.CAMERA;
                }
                break;

            case CAMERA:
                if (badCount >= CAMERA_LOST_LOOPS) {
                    pidf.reset();
                    trackState = TrackState.ODOMETRY;
                }
                break;
        }

        double target;
        switch (trackState) {
            case UNWIND:
                target = clamp(UNWIND_KP * odoErr, UNWIND_MAX_POWER);
                if (Math.abs(target) < UNWIND_MIN_POWER) target = Math.signum(odoErr) * UNWIND_MIN_POWER;
                break;
            case CAMERA:
                target = pidf.calculate(camErr);
                break;
            default:
                target = pidf.calculate(odoErr);
                break;
        }

        // slew limit for smooth state handoffs
        double maxStep = MAX_POWER_SLEW_PER_SEC * dt;
        power += clamp(target - power, maxStep);

        if ((turret.atMaxTicks() && power > 0) || (turret.atMinTicks() && power < 0)) power = 0;
        turret.setPower(power);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end() { turret.setPower(0); }

    private static double clamp(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
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