package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.ProfiledPIDF;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;

/**
 * Motion-profiled DC motor-powered linear slide lift
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class ProfiledLift {
    /**
     * Motor powering the dual lift system
     */
    private final MotorEx[] motors;

    private final ElapsedTime derivTimer = new ElapsedTime();

    public final FIRLowPassFilter accelFilter;
    public final FIRLowPassFilter veloFilter;

    private final VoltageSensor batteryVoltageSensor;

    /**
     * PIDF controller for lift
     */
    public final ProfiledPIDF controller;

    protected String targetPositionName = "Zero";
    protected double currentPosition, currentVelocity, currentAcceleration, targetPosition, maxVelocity, maxAcceleration, kG, INCHES_PER_TICK;
    protected double currentBatteryVoltage = 12.0;

    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update constants
     */
    public ProfiledLift(
            MotorEx[] motors,
            VoltageSensor batteryVoltageSensor,
            ProfiledPIDF controller,
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
     * Update {@link #controller}, {@link #veloFilter}, and {@link #accelFilter} gains
     */
    public void updateConstants(double kG, double INCHES_PER_TICK) {
        this.kG = kG;
        this.INCHES_PER_TICK = INCHES_PER_TICK;
    }

    /**
     * Reads lift encoder value and converts to {@link #currentPosition} in inches
     * Calculates {@link #currentVelocity} and {@link #currentAcceleration} via time-based differentiation
     */
    public void readPosition() {
        double lastPosition = currentPosition;
        double lastVelocity = currentVelocity;
        double timerSeconds = derivTimer.seconds();
        derivTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        currentPosition = motors[0].encoder.getPosition() * INCHES_PER_TICK;
        currentVelocity = veloFilter.getEstimate((currentPosition - lastPosition) / dt);
        currentAcceleration = accelFilter.getEstimate((currentVelocity - lastVelocity) / dt);
        maxVelocity = Math.max(currentVelocity, maxVelocity);
        maxAcceleration = Math.max(currentAcceleration, maxAcceleration);
    }

    /**
     * Set target for lift motion profile
     *
     * @param targetPosition Desired position (in inches) to run to
     */
    public void setTargetPosition(double targetPosition) {
        setTargetPosition(targetPosition, Double.toString(targetPosition));
    }

    /**
     * Set target for lift motion profile
     *
     * @param targetPosition Desired position (in inches) to run to
     */
    public void setTargetPosition(double targetPosition, String targetPositionName) {
        this.targetPosition = targetPosition;
        this.targetPositionName = targetPositionName;
        controller.setTargetPosition(currentPosition, currentVelocity, this.targetPosition);
    }

    /**
     * Resets all internal lift variables
     */
    public void reset() {
        accelFilter.clearMemory();
        veloFilter.clearMemory();

        motors[0].encoder.reset();
        controller.pid.resetIntegral();
        controller.pid.derivFilter.clearMemory();

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

        run(controller.update(currentPosition, currentBatteryVoltage), false);
    }

    /**
     * Run {@link #motors} at the entered percentage of max velocity
     *
     * @param veloCommand       Pass in a velocity command between -1 ≤ x ≤ 1
     * @param voltageCompensate Whether to voltage compensate veloCommand
     */
    public void run(double veloCommand, boolean voltageCompensate) {
        double scalar = 12.0 / currentBatteryVoltage;
        if (voltageCompensate) veloCommand *= scalar;
        for (MotorEx motor : motors) motor.set(kG * scalar + veloCommand);
    }

    /**
     * Print tuning telemetry from {@link #readPosition} and {@link #controller}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Lift current position (in)", currentPosition);
        telemetry.addData("Lift profile position (in)", controller.getProfilePosition());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentVelocity);
        telemetry.addData("Lift profile velocity (in/s)", controller.getProfileVelocity());
        telemetry.addData("Lift max velocity (in/s)", maxVelocity);
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentAcceleration);
        telemetry.addData("Lift max acceleration (in/s^2)", maxAcceleration);
        telemetry.addLine();
        telemetry.addData("Lift error integral (in*s)", controller.pid.getErrorIntegral());
        telemetry.addData("Lift error (in)", controller.pid.getError());
        telemetry.addData("Lift error derivative (in/s)", controller.pid.getErrorVelocity());
    }

    /**
     * Prints user-friendly {@link #targetPositionName} to telemetry
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Named target lift position", targetPositionName);
    }
}
