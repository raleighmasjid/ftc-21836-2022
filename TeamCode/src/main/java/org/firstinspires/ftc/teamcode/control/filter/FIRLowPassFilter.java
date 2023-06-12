package org.firstinspires.ftc.teamcode.control.filter;

import java.util.ArrayList;

/**
 * Finite impulse response low-pass filter;
 * Filters out sensor noise
 */
public class FIRLowPassFilter {
    private double filterGain = 0.8;
    private int pastValuesCount = 5;
    private ArrayList<Double> pastValues = new ArrayList<>();

    public FIRLowPassFilter(double filterGain, int pastValuesCount) {
        setGains(filterGain, pastValuesCount);
        clearMemory();
    }

    public void setGains(double filterGain, int pastValuesCount) {
        this.filterGain = filterGain;
        this.pastValuesCount = (pastValuesCount == 0) ? 1 : pastValuesCount;
    }

    public void clearMemory() {
        pastValues.clear();
        for (int x = 0; x < pastValuesCount; x++) {
            pastValues.add(0.0);
        }
    }

    public double getEstimate(double newValue) {
        double pastValuesTotal = 0.0;
        for (double pastValue : pastValues) pastValuesTotal += pastValue;
        double size = pastValues.size();
        double pastValuesAvg = pastValuesTotal / size;

        pastValues.add(newValue);
        if (size > pastValuesCount) pastValues.remove(0);

        return filterGain * pastValuesAvg + (1 - filterGain) * newValue;
    }
}
