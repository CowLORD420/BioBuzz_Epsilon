package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scheduler.Command;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PIDF;

@Configurable
public class Track implements Command {
    private final Turret turret;
    private final Camera camera;
    private PIDF pidf = new PIDF(0,0,0,0);

    public static double kF = 0.0;
    public static double kD = 0.0;
    public static double kI = 0.0;
    public static double kP = 0.0;

    public Track(Turret turret, Camera camera){
        this.turret = turret;
        this.camera = camera;
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
}
