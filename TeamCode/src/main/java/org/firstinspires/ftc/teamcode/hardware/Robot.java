package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.scheduler.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.GoBildaPinpointDriver;

public class Robot {
    public GamepadWrapper wrapper;
    public CommandScheduler scheduler;
    public Camera camera;
    public Pinpoint pinpoint;
    public Turret turret;
    public Follower follower;

    public enum Color{
        RED,
        BLUE
    }
    public Color color;

    private final HardwareMap hmap;
    private final Gamepad gamepad;

    public Robot(HardwareMap hmap, Gamepad gamepad, Color color){
        this.hmap = hmap;
        this.gamepad = gamepad;
        this.color = color;
    }

    public void init () {
        scheduler = new CommandScheduler();
        wrapper = new GamepadWrapper.Builder(gamepad, scheduler)
                .build();

        turret = new Turret(
                hmap.get(DcMotorEx.class, "turret")
        );

        pinpoint = new Pinpoint(
                hmap.get(GoBildaPinpointDriver.class, "pinpoint")
        );

        camera = new Camera(
                hmap.get(Limelight3A.class, "limelight"),
                color
        );

        follower = Constants.create(hmap);

    }

    public void initRobot(){
        camera.init();
    }

    public void update(){
        camera.update();
        pinpoint.update();
        turret.update();
        scheduler.update();
    }

    public void stop(){
        scheduler.cancelAll();
    }
}
