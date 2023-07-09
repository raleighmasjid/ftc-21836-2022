package org.firstinspires.ftc.teamcode.control.controllers;

public class FullStateController implements FeedbackController {

    private FullStateGainMatrix gainMatrix;

    private double pTarget, vTarget, aTarget;

    public FullStateController(FullStateGainMatrix gainMatrix) {
        setGainMatrix(gainMatrix);
    }

    public void setGainMatrix(FullStateGainMatrix gainMatrix) {
        this.gainMatrix = gainMatrix;
    }

    public void setTargetState(double pTarget, double vTarget, double aTarget) {
        this.pTarget = pTarget;
        this.vTarget = vTarget;
        this.aTarget = aTarget;
    }

    public void setTargetState(double pTarget, double vTarget) {
        setTargetState(pTarget, vTarget, 0.0);
    }

    public void setTargetState(double pTarget) {
        setTargetState(pTarget, 0.0);
    }

    public double update(double pCurrent, double vCurrent, double aCurrent) {
        double pError = pTarget - pCurrent;
        double vError = vTarget - vCurrent;
        double aError = aTarget - aCurrent;

        return (pError * gainMatrix.getPGain()) + (vError * gainMatrix.getVGain()) + (aError * gainMatrix.getAGain());
    }

    public double update(double pCurrent, double vCurrent) {
        return update(pCurrent, vCurrent, 0.0);
    }

    public double update(double pCurrent) {
        return update(pCurrent, 0.0);
    }
}
