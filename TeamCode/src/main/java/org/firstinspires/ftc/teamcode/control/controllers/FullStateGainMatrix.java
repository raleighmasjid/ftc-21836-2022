package org.firstinspires.ftc.teamcode.control.controllers;

public class FullStateGainMatrix {

    private final double pGain, vGain, aGain;

    public FullStateGainMatrix(
            double pGain,
            double vGain,
            double aGain
    ) {
        this.pGain = pGain;
        this.vGain = vGain;
        this.aGain = aGain;
    }

    public double getPGain() {
        return pGain;
    }

    public double getVGain() {
        return vGain;
    }

    public double getAGain() {
        return aGain;
    }
}
