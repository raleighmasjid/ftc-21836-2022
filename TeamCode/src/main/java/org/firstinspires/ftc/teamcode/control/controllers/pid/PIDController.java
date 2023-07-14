package org.firstinspires.ftc.teamcode.control.controllers.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedbackController;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains;
    private State target;

    private double error, errorIntegral, errorDerivative;
    private boolean integrate = true;

    public final FIRLowPassFilter derivFilter;
    private final ElapsedTime dtTimer = new ElapsedTime();

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
        double lastError = error;
        error = target.getX() - measurement.getX();

        if (Math.signum(error) != Math.signum(lastError)) reset();

        double timerSeconds = dtTimer.seconds();
        dtTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        errorDerivative = derivFilter.calculate((error - lastError) / dt);
        if (integrate) errorIntegral += 0.5 * (error + lastError) * dt;

        double output = (gains.getKP() * error) + (gains.getKI() * errorIntegral) + (gains.getKD() * errorDerivative);

        integrate = !(Math.abs(output) > gains.getMaxOutputWithIntegral() && Math.signum(output) == Math.signum(error));

        return output;
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
        errorIntegral = 0.0;
    }
}
