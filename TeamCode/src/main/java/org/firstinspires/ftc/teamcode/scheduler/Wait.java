package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait implements Command {
    private double time;
    private ElapsedTime timer = new ElapsedTime();

    public Wait(double time) {
        this.time = time;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= time;
    }

    @Override
    public void end() {
    }
}