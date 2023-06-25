package org.firstinspires.ftc.teamcode.control.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;

public class PIDController {

    private double kP, kI, kD, maxOutputWithIntegral, lastError, error, errorIntegral, errorVelocity, target;

    private boolean integrate = true;

    private final ElapsedTime dtTimer = new ElapsedTime();

    public FIRLowPassFilter derivFilter;

    public PIDController(double kP, double kI, double kD, double maxOutputWithIntegral, FIRLowPassFilter derivFilter) {
        setGains(kP, kI, kD, maxOutputWithIntegral);
        dtTimer.reset();
        this.derivFilter = derivFilter;
    }

    public void setGains(double kP, double kI, double kD, double maxOutputWithIntegral) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxOutputWithIntegral = maxOutputWithIntegral;
    }

    public double update(double measurement) {
        lastError = error;
        error = target - measurement;

        if (error == 0.0 || Math.signum(error) != Math.signum(lastError)) resetIntegral();

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorVelocity = derivFilter.getEstimate((error - lastError) / dt);
        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;

        double output = (kP * error) + (kI * errorIntegral) + (kD * errorVelocity);

        setIntegrate(!(Math.abs(output) > maxOutputWithIntegral && Math.signum(output) == Math.signum(error)));

        return output;
    }

    public PIDController(double kP, double kI, double kD, double maxOutputWithIntegral) {
        this(kP, kI, kD, maxOutputWithIntegral, new FIRLowPassFilter());
    }

    public PIDController(double kP, double kI, double kD, FIRLowPassFilter derivFilter) {
        this(kP, kI, kD, Double.POSITIVE_INFINITY, derivFilter);
    }

    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, new FIRLowPassFilter());
    }

    public PIDController() {
        this(0, 0, 0);
    }

    public void setGains(double kP, double kI, double kD) {
        setGains(kP, kI, kD, maxOutputWithIntegral);
    }

    public void setIntegrate(boolean integrate) {
        this.integrate = integrate;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return target;
    }

    public double getLastError() {
        return lastError;
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
