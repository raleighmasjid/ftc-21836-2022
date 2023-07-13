package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.coefficients.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains;

    private State target;

    private double lastError, error, errorIntegral, errorDerivative;

    private boolean integrate = true, calculateError = true;

    private final ElapsedTime dtTimer = new ElapsedTime();

    public FIRLowPassFilter derivFilter;

    public PIDController(PIDGains gains, FIRLowPassFilter derivFilter) {
        setGains(gains);
        this.derivFilter = derivFilter;
        dtTimer.reset();
    }

    public PIDController(PIDGains gains) {
        this(gains, new FIRLowPassFilter());
    }

    public PIDController() {
        this(new PIDGains(0, 0, 0));
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        if (calculateError) {
            lastError = error;
            error = target.getX() - measurement.getX();
        } else calculateError = true;

        if (Math.signum(error) != Math.signum(lastError)) resetIntegral();

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorDerivative = derivFilter.getEstimate((error - lastError) / dt);
        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;

        double output = (gains.getKP() * error) + (gains.getKI() * errorIntegral) + (gains.getKD() * errorDerivative);

        setIntegrate(!(Math.abs(output) > gains.getMaxOutputWithIntegral() && Math.signum(output) == Math.signum(error)));

        return output;
    }

    public void setIntegrate(boolean integrate) {
        this.integrate = integrate;
    }

    public void setTarget(State target) {
        this.target = target;
    }

    public double getTarget() {
        return target.getX();
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
