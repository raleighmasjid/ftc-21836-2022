package org.firstinspires.ftc.teamcode.control.controllers.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Differentiator;
import org.firstinspires.ftc.teamcode.control.Integrator;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedbackController;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains;
    private State target;
    private final Differentiator differentiator;
    public final FIRLowPassFilter derivFilter;
    private final Integrator integrator = new Integrator(true);

    private double error, errorIntegral, errorDerivative;

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
        integrator.setMaxOutput(gains.getMaxOutputWithIntegral());
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        error = target.getX() - measurement.getX();

        errorDerivative = differentiator.calculate(error);
        errorIntegral = integrator.calculate(error);

        return (gains.getKP() * error) + (gains.getKI() * errorIntegral) + (gains.getKD() * errorDerivative);
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

    public void reset() {
        integrator.reset();
    }
}
