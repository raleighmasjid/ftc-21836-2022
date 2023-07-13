package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.State;

/**
 * Wrapper class combining a PID controller and a kV-kA-kS feedforward controller.
 */
public class PIDFController implements FeedbackController {
    public final PIDController pid;
    public final FeedforwardController feedforward;

    /**
     * @param pid         PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public PIDFController(PIDController pid, FeedforwardController feedforward) {
        this.pid = pid;
        this.feedforward = feedforward;
    }

    public PIDFController() {
        this(new PIDController(), new FeedforwardController());
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
}
