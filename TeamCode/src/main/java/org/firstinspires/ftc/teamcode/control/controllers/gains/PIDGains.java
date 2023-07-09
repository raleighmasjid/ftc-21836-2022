package org.firstinspires.ftc.teamcode.control.controllers.gains;

public class PIDGains {

    public final double kP, kI, kD;

    public PIDGains(
            double kP,
            double kI,
            double kD
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
