package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains = new FullStateGains();
    private State target = new State(), error = new State();

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
                target.x - measurement.x,
                target.v - measurement.v,
                target.a - measurement.a
        );

        return (error.x * gains.pGain) + (error.x * gains.vGain) + (error.x * gains.aGain);
    }

    public State getError() {
        return error;
    }
}
