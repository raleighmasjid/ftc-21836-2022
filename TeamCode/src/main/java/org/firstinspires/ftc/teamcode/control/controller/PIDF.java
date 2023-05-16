package org.firstinspires.ftc.teamcode.control.controller;

/**
 * PID controller with various feedforward components.
 */
public class PIDF {
    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;
    private double maxIntegrationVelocity;
    public PIDController PID;
    public FeedforwardController Feedforward;

    /**
     * Constructor for [PIDFController]. kV, kA, and kStatic are designed for DC motor feedforward
     * control (the most common kind of feedforward in FTC).
     *
     * @param kP                     proportional gain
     * @param kI                     integral gain
     * @param kD                     derivative gain
     * @param kV                     feedforward velocity gain
     * @param kA                     feedforward acceleration gain
     * @param kStatic                additive feedforward constant
     * @param filterGain             derivative filter weight, 0 = unsmoothed, 0 < x < 1 increasingly smoothed, 1 = broken
     * @param maxIntegrationVelocity max velocity that integral path will continue integration
     */
    public PIDF(
            double kP,
            double kI,
            double kD,
            double filterGain,
            double kV,
            double kA,
            double kStatic,
            double maxIntegrationVelocity
    ) {
        PID = new PIDController(kP, kI, kD, filterGain);
        Feedforward = new FeedforwardController(kV, kA, kStatic);
        this.maxIntegrationVelocity = maxIntegrationVelocity;
    }

    /**
     * Constructor for [PIDFController]. kV, kA, and kStatic are designed for DC motor feedforward
     * control (the most common kind of feedforward in FTC).
     */
    public PIDF() {
        PID = new PIDController(0.0, 0.0, 0.0, 0.8);
        Feedforward = new FeedforwardController(0.0, 0.0, 0.0);
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

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position
     */
    public double update(double measuredPosition) {
        double pidCommand = PID.update(measuredPosition);
        double feedforwardCommand = Feedforward.update(pidCommand);
        double output = pidCommand + feedforwardCommand;

        PID.setIntegrate(Math.abs(output) < maxIntegrationVelocity || Math.signum(output) != Math.signum(PID.getLastError()));

        return (outputBounded) ? Math.max(minOutput, Math.min(output, maxOutput)) : output;
    }
}
