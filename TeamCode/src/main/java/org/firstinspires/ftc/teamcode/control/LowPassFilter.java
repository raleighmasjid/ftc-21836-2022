package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;

public class LowPassFilter {
    private double filterGain = 0.0;
    private int pastEstimatesCount = 5;
    private double lastEstimate = 0.0;
    private final ArrayList<Double> pastEstimates = new ArrayList<>();

    public void setGains (double filterGain, int pastEstimatesCount) {
        this.filterGain = filterGain;
        if (pastEstimatesCount == 0) pastEstimatesCount = 1;
        this.pastEstimatesCount = pastEstimatesCount;
    }

    public void resetPastEstimates() {
        pastEstimates.clear();
        pastEstimates.add(0.0);
    }

    public double getLastEstimate () {
        return lastEstimate;
    }

    public double getEstimate (double measurement) {
        double total = 0.0;
        for (Double estimate : pastEstimates) total += estimate;
        double size = pastEstimates.size();
        lastEstimate = total / size;

        double newEstimate = filterGain * lastEstimate + (1 - filterGain) * measurement;

        pastEstimates.add(newEstimate);
        if (pastEstimates.size() > pastEstimatesCount) pastEstimates.remove(0);

        return newEstimate;
    }
}
