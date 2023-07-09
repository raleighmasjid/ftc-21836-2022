package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.MotionProfiler;

public class ProfiledFullState {

    public FullStateController fullState;
    public FeedforwardController feedforward;
    public MotionProfiler profiler = new MotionProfiler();

    public ProfiledFullState(FullStateController fullState, FeedforwardController feedforward) {
        this.fullState = fullState;
        this.feedforward = feedforward;
    }

    public ProfiledFullState() {
        this(new FullStateController(), new FeedforwardController());
    }

    public double calculate(double pCurrent, double vCurrent, double aCurrent, double voltage) {
        profiler.update();

        fullState.setTargetState(profiler.getX(), profiler.getV(), profiler.getA());
        feedforward.setTargetVelocity(profiler.getV());
        feedforward.setTargetAcceleration(profiler.getA());

        double fullStateOutput = fullState.update(pCurrent, vCurrent, aCurrent);

        return fullStateOutput + feedforward.calculate(voltage, fullStateOutput);
    }

    public double calculate(double pCurrent, double vCurrent, double aCurrent) {
        return this.calculate(pCurrent, vCurrent, aCurrent, 12.0);
    }

    public double calculate(double pCurrent, double vCurrent) {
        return this.calculate(pCurrent, vCurrent, 0.0);
    }

    public double calculate(double pCurrent) {
        return this.calculate(pCurrent, 0.0);
    }
}
