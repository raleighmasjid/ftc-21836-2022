package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.Differentiator;
import org.firstinspires.ftc.teamcode.control.Integrator;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains;
    private State target;

    public final FIRLowPassFilter derivFilter;
    private final Differentiator differentiator;
    private final Integrator integrator = new Integrator(true);

    private double error, errorIntegral, errorDerivative;
    private boolean calculateError = true;

    public PIDController(PIDGains gains, FIRLowPassFilter derivFilter) {
        setGains(gains);
        this.derivFilter = derivFilter;
        differentiator = new Differentiator(derivFilter);
    }

    public PIDController(PIDGains gains) {
        this(gains, new FIRLowPassFilter());
    }

    public PIDController() {
        this(new PIDGains(0, 0, 0));
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
        integrator.setGain(gains.getKI());
        integrator.setMaxOutput(gains.getMaxOutputWithIntegral());
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        if (calculateError) error = target.getX() - measurement.getX();
        else calculateError = true;

        errorDerivative = differentiator.calculate(error);
        errorIntegral = integrator.calculate(error);

        return (gains.getKP() * error) + errorIntegral + (gains.getKD() * errorDerivative);
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
        return errorIntegral;
    }

    public void setError(double error) {
        this.error = error;
        calculateError = false;
    }

    public void reset() {
        integrator.reset();
    }
}
