package org.firstinspires.ftc.teamcode.control.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter;

public class PIDController {

    private double kP, kI, kD, maxIntegrationVelocity;
    private double lastError = 0.0;
    private double error = 0.0;
    private double errorIntegral = 0.0;
    private double errorVelocity = 0.0;
    private double target = 0.0;

    private boolean integrate = true;

    private ElapsedTime dtTimer = new ElapsedTime();

    private IIRLowPassFilter derivFilter;

    public PIDController(double kP, double kI, double kD, double filterGain, double maxIntegrationVelocity) {
        setGains(kP, kI, kD, filterGain, maxIntegrationVelocity);
        dtTimer.reset();
        derivFilter = new IIRLowPassFilter(filterGain);
    }

    public void setGains(double kP, double kI, double kD, double filterGain, double maxIntegrationVelocity) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxIntegrationVelocity = maxIntegrationVelocity;
        derivFilter.setGains(filterGain);
    }

    public double update(double measurement) {
        lastError = error;
        error = target - measurement;

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorVelocity = derivFilter.getEstimate((error - lastError) / dt);
        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;

        double output = (kP * error) + (kI * errorIntegral) + (kD * errorVelocity);

        integrate = !(Math.abs(output) > maxIntegrationVelocity && Math.signum(output) == Math.signum(error));

        return output;
    }

    public void setTarget(double target) {
        this.target = target;
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
