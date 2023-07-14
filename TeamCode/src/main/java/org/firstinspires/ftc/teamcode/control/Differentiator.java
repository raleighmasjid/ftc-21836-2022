package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

public class Differentiator {

    private double lastValue = 0.0, derivative = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public final FIRLowPassFilter filter;

    public Differentiator() {
        this(new FIRLowPassFilter());
    }

    public Differentiator(FIRLowPassFilter filter) {
        this.filter = filter;
    }

    public double calculate(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (dt != 0.0) derivative = filter.calculate((newValue - lastValue) / dt);

        lastValue = newValue;

        return derivative;
    }
}
