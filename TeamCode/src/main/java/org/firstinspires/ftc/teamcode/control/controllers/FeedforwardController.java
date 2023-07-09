package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.control.controllers.gains.FeedforwardGains;

public class FeedforwardController implements Controller {

    private FeedforwardGains gains;

    private MotionState targetState = new MotionState(0, 0, 0);

    public FeedforwardController() {
        this(new FeedforwardGains(0, 0, 0));
    }

    public FeedforwardController(FeedforwardGains gains) {
        setGains(gains);
    }

    public void setGains(FeedforwardGains gains) {
        this.gains = gains;
    }

    public double calculate(double voltage, double additionalOutput) {
        double baseOutput = (gains.kV * targetState.getV()) + (gains.kA * targetState.getA());
        return (Math.signum(baseOutput + additionalOutput) * gains.kStatic + baseOutput) * (12.0 / voltage);
    }

    public double calculate(double voltage) {
        return calculate(voltage, 0.0);
    }

    public double calculate() {
        return calculate(12.0);
    }

    public void setTarget(MotionState targetState) {
        this.targetState = targetState;
    }
}
