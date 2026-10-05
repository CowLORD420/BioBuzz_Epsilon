package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    private final DcMotorEx motor;
    private static double maxTicksLeft = 0;
    private static double maxTicksRight = 0;
    private double ticks;
    private static double TICKS_PER_DEGREE = 0;
    private double robotYaw;
    private static double TICKS_ERROR = 0;

    public Turret(DcMotorEx turretMotor){
        motor = turretMotor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public double getTicks(){
        return ticks;
    }

    public void update(){
        ticks = motor.getCurrentPosition();
    }

    public boolean atMaxTicks(){
        return ticks == maxTicksLeft - TICKS_ERROR || ticks == maxTicksRight - TICKS_ERROR;
    }

    public double getYaw(){
        return Math.toRadians(ticks / TICKS_PER_DEGREE);
    }

    public void setRobotYaw(double robotYaw){
        this.robotYaw = robotYaw;
    }

    public double getRobotRelativeYaw(){
        return wrap(robotYaw + getYaw());
    }

    private static double wrap(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}
