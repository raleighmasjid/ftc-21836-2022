package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.control.controllers.gains.FullStateGainMatrix;

public class FullStateController implements FeedbackController {

    private FullStateGainMatrix gainMatrix;

    private MotionState targetState = new MotionState(0, 0, 0);

    public FullStateController(FullStateGainMatrix gainMatrix) {
        setGainMatrix(gainMatrix);
    }

    public FullStateController() {
        this(new FullStateGainMatrix(0.0, 0.0, 0.0));
    }

    public void setGainMatrix(FullStateGainMatrix gainMatrix) {
        this.gainMatrix = gainMatrix;
    }

    public void setTarget(MotionState targetState) {
        this.targetState = targetState;
    }

    public double calculate(MotionState measuredState) {
        double pError = targetState.getX() - measuredState.getX();
        double vError = targetState.getV() - measuredState.getV();
        double aError = targetState.getA() - measuredState.getA();

        return (pError * gainMatrix.pGain) + (vError * gainMatrix.vGain) + (aError * gainMatrix.aGain);
    }
}
