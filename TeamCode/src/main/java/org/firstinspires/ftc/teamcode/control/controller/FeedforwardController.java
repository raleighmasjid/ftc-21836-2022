package org.firstinspires.ftc.teamcode.control.controller;

public class FeedforwardController {
    private double kV, kA, kS, targetVelocity, targetAcceleration;

    public FeedforwardController(double kV, double kA, double kS) {
        setGains(kV, kA, kS);
    }

    public void setGains(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public double update(double voltage) {
        double baseOutput = (kV * targetVelocity) + (kA * targetAcceleration);
        return (Math.signum(baseOutput) * kS + baseOutput) * (12.0 / voltage);
    }

    public double update() {
        return update(12.0);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setTargetAcceleration(double targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }
}
