package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;

public class LowPassFilter {
    private double filterGain = 0.0;
    private int pastEstimatesCount = 5;
    private double lastEstimate = 0.0;
    private ArrayList<Double> pastEstimates = new ArrayList<Double>();

    public void setGains (double estimateGain, int count) {
        this.filterGain = estimateGain;
        if (count == 0) {
            count = 1;
        }
        pastEstimatesCount = count;
    }

    public void setGains (double filterGain) {
        this.filterGain = filterGain;
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
        for (Double estimate : pastEstimates) {
            total += estimate;
        }
        lastEstimate = total / pastEstimatesCount;

        double newEstimate = filterGain * lastEstimate + (1 - filterGain) * measurement;

        pastEstimates.add(newEstimate);
        if (pastEstimates.size() > pastEstimatesCount) {
            pastEstimates.remove(0);
        }

        return newEstimate;
    }
}
