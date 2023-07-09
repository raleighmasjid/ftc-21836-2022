package org.firstinspires.ftc.teamcode.control.controllers;

public class FeedforwardController {
    private double kV, kA, kS, targetVelocity, targetAcceleration;

    public FeedforwardController() {
        this(0, 0, 0);
    }

    public FeedforwardController(double kV, double kA, double kS) {
        setGains(kV, kA, kS);
    }

    public void setGains(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public double calculate(double voltage, double additionalOutput) {
        double baseOutput = (kV * targetVelocity) + (kA * targetAcceleration);
        return (Math.signum(baseOutput + additionalOutput) * kS + baseOutput) * (12.0 / voltage);
    }

    public double calculate(double voltage) {
        return calculate(voltage, 0.0);
    }

    public double calculate() {
        return calculate(12.0);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setTargetAcceleration(double targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }
}
