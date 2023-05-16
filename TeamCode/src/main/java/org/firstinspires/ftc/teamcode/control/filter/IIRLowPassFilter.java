package org.firstinspires.ftc.teamcode.control.filter;

/**
 * Infinite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class IIRLowPassFilter {
    private double filterGain = 0.0;
    private double lastValue = 0.0;

    public IIRLowPassFilter() {
        setGain(0.8);
    }

    public IIRLowPassFilter(double filterGain) {
        setGain(filterGain);
    }

    public void setGain(double filterGain) {
        this.filterGain = filterGain;
    }

    public void clearMemory() {
        lastValue = 0.0;
    }

    public double getEstimate(double newValue) {
        double estimate = filterGain * lastValue + (1 - filterGain) * newValue;
        lastValue = estimate;
        return estimate;
    }
}