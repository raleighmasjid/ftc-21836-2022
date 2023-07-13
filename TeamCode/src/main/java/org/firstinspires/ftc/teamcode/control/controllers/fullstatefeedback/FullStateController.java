package org.firstinspires.ftc.teamcode.control.controllers.fullstatefeedback;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedbackController;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains;
    private State target;

    @Override
    public void setTarget(State target) {
        this.target = target;
    }

    public void setGains(FullStateGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(State measurement) {
        double pError = target.getX() - measurement.getX();
        double vError = target.getV() - measurement.getV();
        double aError = target.getA() - measurement.getA();

        return (pError * gains.getPGain()) + (vError * gains.getVGain()) + (aError * gains.getAGain());
    }
}
