package org.firstinspires.ftc.teamcode.control.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter;

public class PIDController {

    private double kP, kI, kD, error, errorSum, errorDeriv, target;

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

        errorDeriv = derivFilter.getEstimate((error - lastError) / dt);

        if (integrate) errorSum += 0.5 * (error + lastError) * dt;
        if (Math.signum(error) != Math.signum(lastError)) resetIntegral();

        return (kP * error) + (kI * errorSum) + (kD * errorDeriv);
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

    public double getErrorDeriv() {
        return errorDeriv;
    }

    public double getErrorSum() {
        return errorSum;
    }

    public void resetIntegral() {
        errorSum = 0.0;
    }
}
