package org.firstinspires.ftc.teamcode.utils;

public class PIDF {

    private double kP, kI, kD, kF;

    private double setPoint = 0;
    private double lastError = 0;
    private double lastMeasurement = 0;
    private double integralSum = 0;
    private double lastTimeStamp = 0;
    private boolean firstRun = true;

    // Optional: clamp the integral term to avoid windup
    private double integralMax = Double.MAX_VALUE;
    // Only accumulate integral once error is inside this band (0 = always accumulate)
    private double integralActiveBand = 0;

    // Optional: tolerance for "at target" checks
    private double tolerance = 0.0;

    // Output clamp
    private double outputMin = -1.0;
    private double outputMax = 1.0;

    // Max allowed dt to avoid spikes after loop hiccups (seconds)
    private double maxDeltaTime = 0.1;

    public PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setIntegralMax(double integralMax) {
        this.integralMax = integralMax;
    }

    public void setIntegralActiveBand(double band) {
        this.integralActiveBand = band;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public boolean atSetPoint(double currentPosition) {
        return Math.abs(setPoint - currentPosition) <= tolerance;
    }

    public double calculate(double currentPosition) {
        double currentTime = System.nanoTime() / 1e9;
        double error = setPoint - currentPosition;

        double deltaTime;
        if (firstRun) {
            deltaTime = 0;
            firstRun = false;
        } else {
            deltaTime = currentTime - lastTimeStamp;
            // guard against loop hiccups causing a huge dt spike
            deltaTime = Math.min(deltaTime, maxDeltaTime);
        }

        if (deltaTime > 0 && (integralActiveBand == 0 || Math.abs(error) <= integralActiveBand)) {
            integralSum += error * deltaTime;
            integralSum = clamp(integralSum, -integralMax, integralMax);
        }

        double derivative = (deltaTime > 0) ? -(currentPosition - lastMeasurement) / deltaTime : 0;

        double feedforward = (Math.abs(error) > tolerance) ? kF * Math.signum(error) : 0;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + feedforward;
        output = clamp(output, outputMin, outputMax);

        lastError = error;
        lastMeasurement = currentPosition;
        lastTimeStamp = currentTime;

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastMeasurement = 0;
        firstRun = true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}