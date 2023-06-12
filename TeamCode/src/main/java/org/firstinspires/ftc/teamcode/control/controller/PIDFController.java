package org.firstinspires.ftc.teamcode.control.controller;

/**
 * PID controller with various feedforward components.
 */
public class PIDFController {
    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;
    private double maxIntegrationVelocity;
    public PIDController pid;
    public FeedforwardController feedforward;

    /**
     * @param pid                    PID feedback controller
     * @param feedforward            kV-kA-kS feedforward controller
     * @param maxIntegrationVelocity maximum percentage of motor power at which the integral path will continue integration
     */
    public PIDFController(PIDController pid, FeedforwardController feedforward, double maxIntegrationVelocity) {
        this.pid = pid;
        this.feedforward = feedforward;
        this.maxIntegrationVelocity = maxIntegrationVelocity;
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
            maxIntegrationVelocity = Math.min(maxOutput, maxIntegrationVelocity);
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

        pid.setIntegrate(Math.abs(output) < maxIntegrationVelocity || Math.signum(output) != Math.signum(pid.getError()));

        return (outputBounded) ? Math.max(minOutput, Math.min(output, maxOutput)) : output;
    }

    public void setMaxIntegrationVelocity(double maxIntegrationVelocity) {
        this.maxIntegrationVelocity = outputBounded ? Math.min(maxIntegrationVelocity, maxOutput) : maxIntegrationVelocity;
    }
}
