package org.firstinspires.ftc.teamcode.control.controllers.gains;

public class FullStateGains {

    public final double pGain, vGain, aGain;

    public FullStateGains(
            double pGain,
            double vGain,
            double aGain
    ) {
        this.pGain = pGain;
        this.vGain = vGain;
        this.aGain = aGain;
    }
}
