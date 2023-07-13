package org.firstinspires.ftc.teamcode.control.controllers.pid;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedbackController;
import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

/**
 * Wrapper class combining a PID controller and a kV-kA-kS feedforward controller.
 */
public class PIDVAController implements FeedbackController {
    private final PIDController pid;
    private final FeedforwardController feedforward;
    public FIRLowPassFilter derivFilter;

    /**
     * @param pid         PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public PIDVAController(PIDController pid, FeedforwardController feedforward) {
        this.pid = pid;
        this.feedforward = feedforward;
        derivFilter = this.pid.derivFilter;
    }

    public PIDVAController() {
        this(new PIDController(), new FeedforwardController());
    }

    public void setGains(PIDGains pidGains, FeedforwardGains feedforwardGains) {
        pid.setGains(pidGains);
        feedforward.setGains(feedforwardGains);
    }

    public void setTarget(State target) {
        pid.setTarget(target);
        feedforward.setTarget(target);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        return calculate(measurement, 12.0);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     * @param voltage     measured battery voltage (for feedforward voltage correction)
     */
    public double calculate(State measurement, double voltage) {
        double pidOutput = pid.calculate(measurement);
        return pidOutput + feedforward.calculate(voltage, pidOutput);
    }

    public double getError() {
        return pid.getError();
    }

    public double getErrorDerivative() {
        return pid.getErrorDerivative();
    }

    public double getErrorIntegral() {
        return pid.getErrorIntegral();
    }

    public void reset() {
        pid.reset();
    }
}
