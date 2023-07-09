package org.firstinspires.ftc.teamcode.control.controllers.gains;

public class FullStateGainMatrix {

    public final double pGain, vGain, aGain;

    public FullStateGainMatrix(
            double pGain,
            double vGain,
            double aGain
    ) {
        this.pGain = pGain;
        this.vGain = vGain;
        this.aGain = aGain;
    }
}
