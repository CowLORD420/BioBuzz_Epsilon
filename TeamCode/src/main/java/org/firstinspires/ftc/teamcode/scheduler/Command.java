package org.firstinspires.ftc.teamcode.scheduler;

public interface Command{
    void start();          // called once when scheduled
    void update();         // called every loop
    boolean isFinished();  // when true -> scheduler ends it
    void end();            // cleanup
}