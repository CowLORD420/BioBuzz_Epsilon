package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    private final DcMotorEx motor;
    private static int maxTicksLeft = 0;
    private static int maxTicksRight = 0;
    private int ticks;

    public Turret(DcMotorEx turretMotor){
        motor = turretMotor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public int getTicks(){
        return ticks;
    }

    public void update(){
        ticks = motor.getCurrentPosition();
    }

    public boolean atMaxTicks(){
        if(ticks == maxTicksLeft || ticks == maxTicksRight) return true;
        return false;
    }



}
