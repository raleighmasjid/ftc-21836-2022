package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Integrator {

    private double integral, lastValue, maxOutput;

    private final ElapsedTime timer = new ElapsedTime();

    public Integrator() {
        this(Double.POSITIVE_INFINITY);
    }

    public Integrator(double maxOutput) {
        setMaxOutput(maxOutput);
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double calculate(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (Math.signum(newValue) != Math.signum(lastValue)) reset();

        if (Math.abs(integral) <= maxOutput) integral += 0.5 * (newValue + lastValue) * dt;

        lastValue = newValue;

        return integral;
    }

    public void reset() {
        integral = 0.0;
    }
}
