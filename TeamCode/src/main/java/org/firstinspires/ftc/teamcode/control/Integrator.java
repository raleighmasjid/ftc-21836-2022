package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Integrator {

    private double integral, lastValue, maxOutput = Double.POSITIVE_INFINITY;

    private final ElapsedTime timer = new ElapsedTime();

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double calculate(double newValue) {

        double timerSeconds = timer.seconds();
        timer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        if (Math.signum(newValue) != Math.signum(lastValue)) reset();

        if (Math.abs(integral) <= maxOutput) integral += 0.5 * (newValue + lastValue) * dt;

        lastValue = newValue;

        return integral;
    }

    public void reset() {
        integral = 0.0;
    }
}
