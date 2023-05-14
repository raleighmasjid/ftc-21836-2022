package org.firstinspires.ftc.teamcode.control;

/**
 * Infinite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class IIRLowPassFilter {
    private double filterGain = 0.0;
    private double lastValue = 0.0;

    public IIRLowPassFilter() {
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
        return filterGain * lastValue + (1 - filterGain) * newValue;
    }
}