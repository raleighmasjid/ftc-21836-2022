package org.firstinspires.ftc.teamcode.control.controllers;

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

    public void setTargetState(double targetPosition, double targetVelocity, double targetAcceleration) {
        pid.setTarget(targetPosition);
        feedforward.setTargetVelocity(targetVelocity);
        feedforward.setTargetAcceleration(targetAcceleration);
    }

    public double calculate(double measuredPosition) {
        return update(measuredPosition, 12.0);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param currentPosition measured position
     * @param voltage         measured battery voltage (for feedforward voltage correction)
     */
    public double update(double currentPosition, double voltage) {
        double pidOutput = pid.calculate(currentPosition);
        return pidOutput + feedforward.calculate(voltage, pidOutput);
    }
}
