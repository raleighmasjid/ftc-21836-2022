package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Integrator {

    private double integral = 0.0;
    private boolean integrate = true;

    private final ElapsedTime timer = new ElapsedTime();

    public double calculate(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (integrate) integral += newValue * dt;

        return integral;
    }

    public double getIntegral() {
        return integral;
    }

    public void setIntegrate(boolean integrate) {
        this.integrate = integrate;
    }

    public void reset() {
        integral = 0.0;
    }
}
