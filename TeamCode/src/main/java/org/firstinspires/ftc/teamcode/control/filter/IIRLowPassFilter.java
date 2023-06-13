package org.firstinspires.ftc.teamcode.control.filter;

/**
 * Infinite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class IIRLowPassFilter {
    private double filterGain = 0.0;
    private double estimate = Double.NaN;

    public IIRLowPassFilter(double filterGain) {
        setGains(filterGain);
    }

    public void setGains(double filterGain) {
        this.filterGain = filterGain;
    }

    public void clearMemory() {
        estimate = Double.NaN;
    }

    public double getEstimate(double newValue) {
        estimate = Double.isNaN(estimate) ? newValue : filterGain * estimate + (1 - filterGain) * newValue;
        return estimate;
    }
}