package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double kP, kI, kD, target, lastError, errorIntegral;

    private final ElapsedTime timer = new ElapsedTime();

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double calculate(double measurement) {
        double error = target - measurement;
        double dt = timer.seconds();
        timer.reset();

        if (Math.signum(error) != Math.signum(lastError)) errorIntegral = 0.0;
        errorIntegral += error * dt;
        double errorDerivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);
    }
}
