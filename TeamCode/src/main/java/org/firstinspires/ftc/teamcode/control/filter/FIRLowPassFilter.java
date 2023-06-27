package org.firstinspires.ftc.teamcode.control.filter;

import java.util.ArrayList;

/**
 * Finite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class FIRLowPassFilter implements Filter {
    private double filterGain;
    private int filterCount;
    private ArrayList<Double> values = new ArrayList<>();

    public FIRLowPassFilter() {
        this(0.8, 300);
    }

    public FIRLowPassFilter(double filterGain, int filterCount) {
        setGains(filterGain, filterCount);
    }

    public void setGains(double filterGain, int filterCount) {
        this.filterGain = filterGain;
        this.filterCount = Math.max(filterCount, 2);
    }

    public void clearMemory() {
        values.clear();
    }

    public double getEstimate(double newValue) {
        values.add(newValue);
        if (values.size() < 2) {
            return newValue;
        }
        while (values.size() > filterCount) {
            values.remove(0);
        }

        double estimate = values.get(0);
        for (int ind = 1; ind < values.size(); ind++) {
            estimate = filterGain * estimate + (1 - filterGain) * values.get(ind);
        }

        return estimate;
    }
}
