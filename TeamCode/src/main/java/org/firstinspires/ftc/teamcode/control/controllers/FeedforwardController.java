package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;

public class FeedforwardController implements Controller {

    private FeedforwardGains gains;

    private State target;

    public FeedforwardController() {
        this(new FeedforwardGains(0.0, 0.0, 0.0));
    }

    public FeedforwardController(FeedforwardGains gains) {
        setGains(gains);
    }

    public void setGains(FeedforwardGains gains) {
        this.gains = gains;
    }

    public double calculate(double voltage, double additionalOutput) {
        double baseOutput = (gains.kV * target.v) + (gains.kA * target.a);
        return (Math.signum(baseOutput + additionalOutput) * gains.kStatic + baseOutput) * (12.0 / voltage);
    }

    public double calculate(double voltage) {
        return calculate(voltage, 0.0);
    }

    public double calculate() {
        return calculate(12.0);
    }

    /**
     * @param target The V and A attributes of the {@link State} parameter are used as velocity and acceleration references
     */
    public void setTarget(State target) {
        this.target = target;
    }
}
