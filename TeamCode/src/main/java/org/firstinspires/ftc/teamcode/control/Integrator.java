package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Integrator {

    private double integral = 0.0, lastValue = 0.0, gain = 0.1, maxOutput = Double.POSITIVE_INFINITY;
    private final boolean resetOnSignChange;

    private final ElapsedTime timer = new ElapsedTime();

    public Integrator() {
        this(false);
    }

    public Integrator(boolean resetOnSignChange) {
        this.resetOnSignChange = resetOnSignChange;
    }

    public void setGain(double gain) {
        this.gain = gain;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double calculate(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (resetOnSignChange && (Math.signum(newValue) != Math.signum(lastValue))) reset();

        if (Math.abs(integral * gain) <= maxOutput) integral += 0.5 * (newValue + lastValue) * dt;

        lastValue = newValue;

        return integral * gain;
    }

    public void reset() {
        integral = 0.0;
    }

    public double getIntegral() {
        return integral;
    }
}
