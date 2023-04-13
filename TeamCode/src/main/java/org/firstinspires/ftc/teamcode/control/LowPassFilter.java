package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;

public class LowPassFilter {
    private double filterGain = 0.0;
    private int pastValuesCount = 5;
    private final ArrayList<Double> pastValues = new ArrayList<>();

    public void setGains (double filterGain, int pastValuesCount) {
        if (pastValuesCount == 0) pastValuesCount = 1;
        this.filterGain = filterGain;
        this.pastValuesCount = pastValuesCount;
    }

    public void resetPastValues () {
        pastValues.clear();
        pastValues.add(0.0);
    }

    public double getEstimate (double newValue) {
        double pastValuesAvg = 0.0;
        for (Double pastValue : pastValues) pastValuesAvg += pastValue;
        double size = pastValues.size();
        pastValuesAvg = pastValuesAvg / size;

        double estimate = filterGain * pastValuesAvg + (1 - filterGain) * newValue;

        pastValues.add(newValue);
        if (pastValues.size() > pastValuesCount) pastValues.remove(0);

        return estimate;
    }
}
