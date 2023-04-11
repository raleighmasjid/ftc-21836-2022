package org.firstinspires.ftc.teamcode.control;

public class LowPassFilter {
    private double filterGain = 0.0;
    private double lastEstimate = 0.0;

    public void setGains (double filterGain) {
        this.filterGain = filterGain;
    }

    public void resetLastEstimate () {
        lastEstimate = 0.0;
    }

    public double getLastEstimate () {
        return lastEstimate;
    }

    public double getEstimate (double measurement) {
        double output = filterGain * lastEstimate + (1 - filterGain) * measurement;
        lastEstimate = measurement;
        return output;
    }
}
