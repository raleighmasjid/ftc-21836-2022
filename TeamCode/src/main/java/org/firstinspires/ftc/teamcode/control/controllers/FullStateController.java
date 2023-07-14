package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FullStateGains;

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
                target.getX() - measurement.getX(),
                target.getV() - measurement.getV(),
                target.getA() - measurement.getA()
        );

        return (error.getX() * gains.pGain) + (error.getV() * gains.vGain) + (error.getA() * gains.aGain);
    }

    public State getError() {
        return error;
    }
}
