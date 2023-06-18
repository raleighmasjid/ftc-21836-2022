package org.firstinspires.ftc.teamcode.control.controller;

/**
 * Wrapper class combining a PID controller and a kV-kA-kS feedforward controller.
 */
public class PIDFController {
    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;
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

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    public void setOutputBounds(double min, double max) {
        if (min != max) {
            outputBounded = true;
            minOutput = Math.min(min, max);
            maxOutput = Math.max(min, max);
        }
    }

    public double update(double measuredPosition) {
        return update(measuredPosition, 12.0);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position
     * @param voltage          measured battery voltage (for feedforward voltage correction)
     */
    public double update(double measuredPosition, double voltage) {
        double output = pid.update(measuredPosition) + feedforward.update(voltage);

        return (outputBounded) ? Math.max(minOutput, Math.min(output, maxOutput)) : output;
    }
}
