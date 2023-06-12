package org.firstinspires.ftc.teamcode.control.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter;

public class PIDController {

    private double kP, kI, kD, error, errorIntegral, errorVelocity, target;

    private boolean integrate = true;

    private final ElapsedTime dtTimer;

    private final IIRLowPassFilter derivFilter;

    public PIDController(double kP, double kI, double kD, double filterGain) {
        setGains(kP, kI, kD, filterGain);
        dtTimer = new ElapsedTime();
        derivFilter = new IIRLowPassFilter(filterGain);
    }

    public void setGains(double kP, double kI, double kD, double filterGain) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        derivFilter.setGain(filterGain);
    }

    public double update(double measurement) {
        double lastError = error;
        error = target - measurement;

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorVelocity = derivFilter.getEstimate((error - lastError) / dt);

        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;
        if (Math.signum(error) != Math.signum(lastError)) resetIntegral();

        return (kP * error) + (kI * errorIntegral) + (kD * errorVelocity);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setIntegrate(boolean integrate) {
        this.integrate = integrate;
    }

    public double getError() {
        return error;
    }

    public double getErrorVelocity() {
        return errorVelocity;
    }

    public double getErrorIntegral() {
        return errorIntegral;
    }

    public void resetIntegral() {
        errorIntegral = 0.0;
    }
}
