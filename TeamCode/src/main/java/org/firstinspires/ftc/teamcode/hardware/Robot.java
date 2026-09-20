package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.scheduler.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

public class Robot {
    public GamepadWrapper wrapper;
    public CommandScheduler scheduler;

    private final HardwareMap hmap;
    private final Gamepad gamepad;

    public Robot(HardwareMap hmap, Gamepad gamepad){
        this.hmap = hmap;
        this.gamepad = gamepad;
    }

    public void init () {
        scheduler = new CommandScheduler();

        wrapper = new GamepadWrapper.Builder(gamepad, scheduler)
                .build();

    }

    public void initRobot(){
    }

    public void update(){
        scheduler.update();
    }

    public void stop(){
        scheduler.cancelAll();
    }
}
