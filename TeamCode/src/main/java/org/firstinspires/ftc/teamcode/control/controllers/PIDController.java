package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.gains.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains;

    private double maxOutputWithIntegral, lastError, error, errorIntegral, errorDerivative, target;

    private boolean integrate = true, calculateError = true;

    private final ElapsedTime dtTimer = new ElapsedTime();

    public FIRLowPassFilter derivFilter;

    public PIDController(PIDGains gains, double maxOutputWithIntegral, FIRLowPassFilter derivFilter) {
        setGains(gains, maxOutputWithIntegral);
        dtTimer.reset();
        this.derivFilter = derivFilter;
    }

    public PIDController() {
        this(new PIDGains(0, 0, 0), Double.POSITIVE_INFINITY, new FIRLowPassFilter());
    }

    public void setGains(PIDGains gains, double maxOutputWithIntegral) {
        this.gains = gains;
        this.maxOutputWithIntegral = maxOutputWithIntegral;
    }

    public void setGains(PIDGains gains) {
        setGains(gains, maxOutputWithIntegral);
    }

    public double calculate(MotionState measuredState) {
        if (calculateError) {
            lastError = error;
            error = target - measuredState.getX();
        } else calculateError = true;

        if (Math.signum(error) != Math.signum(lastError)) resetIntegral();

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorDerivative = derivFilter.getEstimate((error - lastError) / dt);
        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;

        double output = (gains.kP * error) + (gains.kI * errorIntegral) + (gains.kD * errorDerivative);

        setIntegrate(!(Math.abs(output) > maxOutputWithIntegral && Math.signum(output) == Math.signum(error)));

        return output;
    }

    public void setIntegrate(boolean integrate) {
        this.integrate = integrate;
    }

    public void setTarget(MotionState targetState) {
        this.target = targetState.getX();
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
