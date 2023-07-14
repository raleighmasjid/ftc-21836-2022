package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class Differentiator {

    private double lastValue;

    private final ElapsedTime timer = new ElapsedTime();

    public final FIRLowPassFilter filter = new FIRLowPassFilter();

    public double calculate(double newValue) {
        double output = filter.calculate((newValue - lastValue) / timer.seconds());
        timer.reset();
        lastValue = newValue;
        return output;
    }
}
