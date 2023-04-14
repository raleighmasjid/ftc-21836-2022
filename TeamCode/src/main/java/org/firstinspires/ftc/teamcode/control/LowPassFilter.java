package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;

public class LowPassFilter {
    private double filterGain = 0.0;
    private int pastValuesCount = 5;
    private final ArrayList<Double> pastValues = new ArrayList<>();

    public LowPassFilter () {
        resetPastValues();
    }

    public void setGains (double filterGain, int pastValuesCount) {
        if (pastValuesCount == 0) pastValuesCount = 1;
        this.filterGain = filterGain;
        this.pastValuesCount = pastValuesCount;
    }

    public void resetPastValues () {
        pastValues.clear();
        for (int x = 0; x < pastValuesCount; x++) pastValues.add(0.0);
    }

    public double getEstimate (double newValue) {
        double pastValuesTotal = 0.0;
        for (Double pastValue : pastValues) pastValuesTotal += pastValue;
        double size = pastValues.size();
        double pastValuesAvg = pastValuesTotal / size;

        pastValues.add(newValue);
        if (size > pastValuesCount) pastValues.remove(0);

        return filterGain * pastValuesAvg + (1 - filterGain) * newValue;
    }
}
