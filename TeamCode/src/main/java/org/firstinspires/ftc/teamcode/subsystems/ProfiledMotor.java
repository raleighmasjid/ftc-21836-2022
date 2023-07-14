package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Integrator;
import org.firstinspires.ftc.teamcode.control.MotionProfiler;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controllers.FullStateController;
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

    public final Integrator integrator;
    public final FullStateController fullState;
    public final FeedforwardController feedforward;

    public final MotionProfiler profiler = new MotionProfiler();

    protected String targetPositionName = "Zero";

    protected double targetPosition, maxVelocity, maxAcceleration, UNIT_PER_TICK, currentBatteryVoltage = 12.0;

    protected State currentState;

    public double getCurrentPosition() {
        return currentState.getX();
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update constants
     */
    public ProfiledMotor(
            MotorEx[] motors,
            VoltageSensor batteryVoltageSensor,
            Integrator integrator,
            FullStateController fullState,
            FeedforwardController feedforward,
            FIRLowPassFilter veloFilter,
            FIRLowPassFilter accelFilter
    ) {
        this.motors = motors;
        for (MotorEx motor : this.motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }
        this.batteryVoltageSensor = batteryVoltageSensor;
        this.integrator = integrator;
        this.fullState = fullState;
        this.feedforward = feedforward;
        this.veloFilter = veloFilter;
        this.accelFilter = accelFilter;

        derivTimer.reset();

        reset();
    }

    /**
     * @param UNIT_PER_TICK Arbitrary unit per tick scale factor
     */
    public void updateConstants(double UNIT_PER_TICK, double integralGain) {
        this.UNIT_PER_TICK = UNIT_PER_TICK;
        integrator.setGain(integralGain);
    }

    /**
     * Reads encoder value and converts to inches
     */
    public void readPosition() {
        State lastState = currentState;
        double timerSeconds = derivTimer.seconds();
        derivTimer.reset();
        double dt = timerSeconds == 0 ? 0.002 : timerSeconds;

        currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        double x = motors[0].encoder.getPosition() * UNIT_PER_TICK;
        double v = veloFilter.calculate((x - lastState.getX()) / dt);
        double a = accelFilter.calculate((v - lastState.getV()) / dt);
        currentState = new State(x, v, a);
        maxVelocity = Math.max(currentState.getV(), maxVelocity);
        maxAcceleration = Math.max(currentState.getA(), maxAcceleration);
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
        profiler.generateProfile(
                currentState,
                new State(targetPosition)
        );
    }

    /**
     * Resets internal states to 0
     */
    public void reset() {
        accelFilter.reset();
        veloFilter.reset();

        motors[0].encoder.reset();
        integrator.reset();

        currentState = new State();
        maxVelocity = 0.0;
        maxAcceleration = 0.0;

        targetPosition = 0.0;
        targetPositionName = "Zero";

        setTargetPosition(targetPosition, targetPositionName);
    }

    /**
     * Runs {@link #integrator}
     */
    public void runToPosition() {

        profiler.update();
        State setpoint = new State(profiler.getX(), profiler.getV(), profiler.getA());
        fullState.setTarget(setpoint);
        feedforward.setTarget(setpoint);

        double fullStateOutput = fullState.calculate(currentState);
        double feedforwardOutput = feedforward.calculate(currentBatteryVoltage, fullStateOutput);
        double integratorOutput = integrator.calculate(fullState.getError().getX());

        run((fullStateOutput + feedforwardOutput + integratorOutput), false);
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
     * Print tuning telemetry from {@link #readPosition} and {@link #integrator}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current position (in)", currentState.getX());
        telemetry.addData("Profile position (in)", profiler.getX());
        telemetry.addLine();
        telemetry.addData("Current velocity (in/s)", currentState.getV());
        telemetry.addData("Profile velocity (in/s)", profiler.getV());
        telemetry.addData("Max velocity (in/s)", maxVelocity);
        telemetry.addLine();
        telemetry.addData("Current acceleration (in/s^2)", currentState.getA());
        telemetry.addData("Profile acceleration (in/s^2)", profiler.getA());
        telemetry.addData("Max acceleration (in/s^2)", maxAcceleration);
        telemetry.addLine();
        telemetry.addData("Position error integral (in*s)", integrator.getIntegral());
        telemetry.addData("Position error (in)", fullState.getError().getX());
        telemetry.addData("Velocity error (in/s)", fullState.getError().getV());
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
