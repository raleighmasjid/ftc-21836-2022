package org.firstinspires.ftc.teamcode.control.controllers.fullstatefeedback;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedbackController;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains;
    private State target, error;

    @Override
    public void setTarget(State target) {
        this.target = target;
    }

    public void setGains(FullStateGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(State measurement) {
        error = new State(
                target.getX() - measurement.getX(),
                target.getV() - measurement.getV(),
                target.getA() - measurement.getA()
        );

        return (error.getX() * gains.getPGain()) + (error.getV() * gains.getVGain()) + (error.getA() * gains.getAGain());
    }

    public State getError() {
        return error;
    }
}
