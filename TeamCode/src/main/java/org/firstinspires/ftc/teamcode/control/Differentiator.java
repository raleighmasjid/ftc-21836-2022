package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class Differentiator extends FIRLowPassFilter {

    private double lastValue;

    private final ElapsedTime timer;

    public Differentiator() {
        this(0.5, 10);
    }

    public Differentiator(double filterGain, int filterCount) {
        super.setGains(filterGain, filterCount);
        timer = new ElapsedTime();
    }

    @Override
    public double calculate(double newValue) {
        double output = super.calculate((newValue - lastValue) / timer.seconds());
        timer.reset();
        lastValue = newValue;
        return output;
    }
}
