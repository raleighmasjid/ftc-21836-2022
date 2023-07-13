package org.firstinspires.ftc.teamcode.control.controllers.fullstatefeedback;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedbackController;
import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FullStateGains;

/**
 * Wrapper class combining a PID controller and a kV-kA-kS feedforward controller.
 */
public class FullStateVAController implements FeedbackController {
    private final FullStateController fullState;
    private final FeedforwardController feedforward;

    /**
     * @param fullState   PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public FullStateVAController(FullStateController fullState, FeedforwardController feedforward) {
        this.fullState = fullState;
        this.feedforward = feedforward;
    }

    public FullStateVAController() {
        this(new FullStateController(), new FeedforwardController());
    }

    public void setGains(FullStateGains pidGains, FeedforwardGains feedforwardGains) {
        fullState.setGains(pidGains);
        feedforward.setGains(feedforwardGains);
    }

    public void setTarget(State target) {
        fullState.setTarget(target);
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
        double fullStateOutput = fullState.calculate(measurement);
        return fullStateOutput + feedforward.calculate(voltage, fullStateOutput);
    }
}
