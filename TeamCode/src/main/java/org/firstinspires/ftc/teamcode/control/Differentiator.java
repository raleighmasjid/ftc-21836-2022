package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class Differentiator {

    private double lastValue, derivative;

    private final ElapsedTime timer = new ElapsedTime();

    public final FIRLowPassFilter filter = new FIRLowPassFilter();

    public double calculate(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (dt != 0.0) derivative = filter.calculate((newValue - lastValue) / dt);

        lastValue = newValue;

        return derivative;
    }
}
