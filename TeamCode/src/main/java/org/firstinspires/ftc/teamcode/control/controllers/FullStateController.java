package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.control.controllers.gains.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains;

    private MotionState targetState = new MotionState(0, 0, 0);

    public FullStateController(FullStateGains gains) {
        setGains(gains);
    }

    public FullStateController() {
        this(new FullStateGains(0.0, 0.0, 0.0));
    }

    public void setGains(FullStateGains gains) {
        this.gains = gains;
    }

    public void setTarget(MotionState targetState) {
        this.targetState = targetState;
    }

    public double calculate(MotionState measuredState) {
        double pError = targetState.getX() - measuredState.getX();
        double vError = targetState.getV() - measuredState.getV();
        double aError = targetState.getA() - measuredState.getA();

        return (pError * gains.pGain) + (vError * gains.vGain) + (aError * gains.aGain);
    }
}
