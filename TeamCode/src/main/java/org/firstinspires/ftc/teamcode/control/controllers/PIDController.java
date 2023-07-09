package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private double kP, kI, kD, maxOutputWithIntegral, lastError, error, errorIntegral, errorDerivative, target;

    private boolean integrate = true, calculateError = true;

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

    public double calculate(double measurement) {
        if (calculateError) {
            lastError = error;
            error = target - measurement;
        } else calculateError = true;

        if (Math.signum(error) != Math.signum(lastError)) resetIntegral();

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorDerivative = derivFilter.getEstimate((error - lastError) / dt);
        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;

        double output = (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);

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

    public void setError(double error) {
        lastError = this.error;
        this.error = error;
        calculateError = false;
    }

    public double getErrorDerivative() {
        return errorDerivative;
    }

    public double getErrorIntegral() {
        return errorIntegral;
    }

    public void resetIntegral() {
        errorIntegral = 0.0;
    }
}
