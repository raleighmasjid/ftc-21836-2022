package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.Differentiator;
import org.firstinspires.ftc.teamcode.control.Integrator;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class PIDController implements FeedbackController {

    private PIDGains gains = new PIDGains();
    private State target = new State();

    public final FIRLowPassFilter derivFilter;
    private final Differentiator differentiator;
    private final Integrator integrator = new Integrator();

    private double error, lastError;
    private boolean calculateError = true;

    public PIDController(FIRLowPassFilter derivFilter) {
        this.derivFilter = derivFilter;
        differentiator = new Differentiator(derivFilter);
    }

    public PIDController() {
        this(new FIRLowPassFilter());
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        if (calculateError) error = target.x - measurement.x;
        else calculateError = true;

        double errorIntegral = integrator.getIntegral(error);
        if (Math.signum(error) != Math.signum(lastError)) reset();
        lastError = error;

        double output = (gains.kP * error) + (gains.kI * errorIntegral) + (gains.kD * differentiator.getDerivative(error));

        integrator.stopIntegration(Math.abs(output) >= gains.maxOutputWithIntegral && Math.signum(output) == Math.signum(error));

        return output;
    }

    public void setTarget(State target) {
        this.target = target;
    }

    public State getError() {
        return new State(error);
    }

    public double getErrorDerivative() {
        return differentiator.getDerivative();
    }

    public double getErrorIntegral() {
        return integrator.getIntegral();
    }

    public void setError(double error) {
        this.error = error;
        calculateError = false;
    }

    public void reset() {
        integrator.reset();
    }
}
