package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.pid.ProfiledPIDVA;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

/**
 * Motion-profiled DC motor(s)
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class ProfiledMotor {

    private final MotorEx[] motors;

    private final ElapsedTime derivTimer = new ElapsedTime();

    public final FIRLowPassFilter accelFilter;
    public final FIRLowPassFilter veloFilter;

    private final VoltageSensor batteryVoltageSensor;

    public final ProfiledPIDVA controller;

    protected String targetPositionName = "Zero";

    protected double currentPosition, currentVelocity, currentAcceleration, targetPosition, maxVelocity, maxAcceleration, UNIT_PER_TICK, currentBatteryVoltage = 12.0;

    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateScale} to update constants
     */
    public ProfiledMotor(
            MotorEx[] motors,
            VoltageSensor batteryVoltageSensor,
            ProfiledPIDVA controller,
            FIRLowPassFilter veloFilter,
            FIRLowPassFilter accelFilter
    ) {
        this.motors = motors;
        for (MotorEx motor : this.motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }
        this.batteryVoltageSensor = batteryVoltageSensor;
        this.controller = controller;
        this.veloFilter = veloFilter;
        this.accelFilter = accelFilter;

        derivTimer.reset();

        reset();
    }

    /**
     * @param UNIT_PER_TICK Arbitrary unit per tick scale factor
     */
    public void updateScale(double UNIT_PER_TICK) {
        this.UNIT_PER_TICK = UNIT_PER_TICK;
    }

    /**
     * Reads encoder value and converts to {@link #currentPosition} in inches
     * Calculates {@link #currentVelocity} and {@link #currentAcceleration} via time-based differentiation
     */
    public void readPosition() {
        double lastPosition = currentPosition;
        double lastVelocity = currentVelocity;
        double timerSeconds = derivTimer.seconds();
        derivTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        currentPosition = motors[0].encoder.getPosition() * UNIT_PER_TICK;
        currentVelocity = veloFilter.getEstimate((currentPosition - lastPosition) / dt);
        currentAcceleration = accelFilter.getEstimate((currentVelocity - lastVelocity) / dt);
        maxVelocity = Math.max(currentVelocity, maxVelocity);
        maxAcceleration = Math.max(currentAcceleration, maxAcceleration);
    }

    /**
     * Set target for motion profile
     *
     * @param targetPosition Desired position (in inches) to run to
     */
    public void setTargetPosition(double targetPosition) {
        setTargetPosition(targetPosition, Double.toString(targetPosition));
    }

    /**
     * Set target for motion profile
     *
     * @param targetPosition Desired position (in inches) to run to
     */
    public void setTargetPosition(double targetPosition, String targetPositionName) {
        this.targetPosition = targetPosition;
        this.targetPositionName = targetPositionName;
        controller.setTarget(new State(currentPosition, currentVelocity), new State(this.targetPosition));
    }

    /**
     * Resets internal states to 0
     */
    public void reset() {
        accelFilter.reset();
        veloFilter.reset();

        motors[0].encoder.reset();
        controller.reset();
        controller.derivFilter.reset();

        currentPosition = 0.0;
        currentVelocity = 0.0;
        currentAcceleration = 0.0;
        maxVelocity = 0.0;
        maxAcceleration = 0.0;

        targetPosition = 0.0;
        targetPositionName = "Zero";

        setTargetPosition(targetPosition, targetPositionName);
    }

    /**
     * Runs {@link #controller}
     */
    public void runToPosition() {
        run(controller.calculate(new State(currentPosition), currentBatteryVoltage), false);
    }

    /**
     * Run {@link #motors} at the entered percentage of max velocity
     *
     * @param veloCommand       Pass in a velocity command between -1 ≤ x ≤ 1
     * @param voltageCompensate Whether to voltage compensate veloCommand
     */
    public void run(double veloCommand, boolean voltageCompensate) {
        if (voltageCompensate) veloCommand *= 12.0 / currentBatteryVoltage;
        for (MotorEx motor : motors) motor.set(veloCommand);
    }

    /**
     * Print tuning telemetry from {@link #readPosition} and {@link #controller}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current position (in)", currentPosition);
        telemetry.addData("Profile position (in)", controller.getX());
        telemetry.addLine();
        telemetry.addData("Current velocity (in/s)", currentVelocity);
        telemetry.addData("Profile velocity (in/s)", controller.getV());
        telemetry.addData("Max velocity (in/s)", maxVelocity);
        telemetry.addLine();
        telemetry.addData("Current acceleration (in/s^2)", currentAcceleration);
        telemetry.addData("Max acceleration (in/s^2)", maxAcceleration);
        telemetry.addLine();
        telemetry.addData("Error integral (in*s)", controller.getErrorIntegral());
        telemetry.addData("Error (in)", controller.getError().getX());
        telemetry.addData("Error derivative (in/s)", controller.getErrorDerivative());
    }

    /**
     * Prints user-friendly {@link #targetPositionName} to telemetry
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Named target position", targetPositionName);
    }
}
