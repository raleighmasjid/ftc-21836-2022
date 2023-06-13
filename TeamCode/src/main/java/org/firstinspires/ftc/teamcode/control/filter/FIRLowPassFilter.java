package org.firstinspires.ftc.teamcode.control.filter;

import java.util.ArrayList;

/**
 * Finite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class FIRLowPassFilter {
    private double filterGain;
    private int filterCount;
    private ArrayList<Double> pastValues = new ArrayList<>();

    public FIRLowPassFilter(double filterGain, int filterCount) {
        setGains(filterGain, filterCount);
    }

    public void setGains(double filterGain, int filterCount) {
        this.filterGain = filterGain;
        this.filterCount = Math.max(filterCount, 2);
    }

    public void clearMemory() {
        pastValues.clear();
    }

    public double getEstimate(double newValue) {
        pastValues.add(newValue);
        if (pastValues.size() > filterCount) {
            pastValues.remove(0);
        } else if (pastValues.size() < 2) {
            return newValue;
        }

        double estimate = pastValues.get(0);
        for (int ind = 1; ind < pastValues.size(); ind++) {
            estimate = filterGain * estimate + (1 - filterGain) * pastValues.get(ind);
        }

        return estimate;
    }
}
