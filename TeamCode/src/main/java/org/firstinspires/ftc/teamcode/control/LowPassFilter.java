package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;

public class LowPassFilter {
    private double filterGain = 0.0;
    private int pastEstimatesCount = 5;
    private final ArrayList<Double> pastEstimates = new ArrayList<>();

    public void setGains (double filterGain, int pastEstimatesCount) {
        if (pastEstimatesCount == 0) pastEstimatesCount = 1;
        this.filterGain = filterGain;
        this.pastEstimatesCount = pastEstimatesCount;
    }

    public void resetPastEstimates() {
        pastEstimates.clear();
        pastEstimates.add(0.0);
    }

    public double getEstimate (double measurement) {
        double estimateAvg = 0.0;
        for (Double estimate : pastEstimates) estimateAvg += estimate;
        double size = pastEstimates.size();
        estimateAvg = estimateAvg / size;

        double newEstimate = filterGain * estimateAvg + (1 - filterGain) * measurement;

        pastEstimates.add(newEstimate);
        if (pastEstimates.size() > pastEstimatesCount) pastEstimates.remove(0);

        return newEstimate;
    }
}
