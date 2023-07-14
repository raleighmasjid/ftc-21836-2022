package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.Differentiator;
import org.firstinspires.ftc.teamcode.control.Integrator;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains = new PIDGains();
    private State target = new State();

    public final FIRLowPassFilter derivFilter;
    private final Differentiator differentiator;
    private final Integrator integrator = new Integrator(true);

    private double error;
    private double errorDerivative;
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
        integrator.setGain(gains.kI);
        integrator.setMaxOutput(gains.maxOutputWithIntegral);
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        if (calculateError) error = target.getX() - measurement.getX();
        else calculateError = true;

        errorDerivative = differentiator.calculate(error);

        return (gains.kP * error) + integrator.calculate(error) + (gains.kD * errorDerivative);
    }

    public void setTarget(State target) {
        this.target = target;
    }

    public State getError() {
        return new State(error);
    }

    public double getErrorDerivative() {
        return errorDerivative;
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
