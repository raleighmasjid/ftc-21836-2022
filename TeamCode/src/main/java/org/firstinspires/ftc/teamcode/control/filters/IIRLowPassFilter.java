package org.firstinspires.ftc.teamcode.control.filters;

/**
 * Infinite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class IIRLowPassFilter implements Filter {
    private double filterGain = 0.0;
    private double estimate = Double.NaN;

    public IIRLowPassFilter(double filterGain) {
        setGains(filterGain);
    }

    public void setGains(double filterGain) {
        this.filterGain = filterGain;
    }

    public void reset() {
        estimate = Double.NaN;
    }

    public double calculate(double newValue) {
        estimate = Double.isNaN(estimate) ? newValue : filterGain * estimate + (1 - filterGain) * newValue;
        return estimate;
    }
}