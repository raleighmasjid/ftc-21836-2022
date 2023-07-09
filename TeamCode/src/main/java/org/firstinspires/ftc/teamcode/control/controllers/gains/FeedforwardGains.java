package org.firstinspires.ftc.teamcode.control.controllers.gains;

public class FeedforwardGains {

    public final double kV, kA, kStatic;

    public FeedforwardGains(
            double kV,
            double kA,
            double kStatic
    ) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }
}
