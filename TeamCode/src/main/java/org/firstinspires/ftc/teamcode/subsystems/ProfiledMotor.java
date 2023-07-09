package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.controllers.ProfiledController;
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

    /**
     * PIDF controller
     */
    public final ProfiledController controller;
    protected final PIDController pid;
    protected final FeedforwardController feedforward;

    protected String targetPositionName = "Zero";

    protected double currentPosition, currentVelocity, currentAcceleration, targetPosition, maxVelocity, maxAcceleration, kG, UNIT_PER_TICK, currentBatteryVoltage = 12.0;

    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update constants
     */
    public ProfiledMotor(
            MotorEx[] motors,
            VoltageSensor batteryVoltageSensor,
            PIDController pid,
            FeedforwardController feedforward,
            FIRLowPassFilter veloFilter,
            FIRLowPassFilter accelFilter
    ) {
        this.motors = motors;
        for (MotorEx motor : this.motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }
        this.batteryVoltageSensor = batteryVoltageSensor;
        this.pid = pid;
        this.feedforward = feedforward;
        this.controller = new ProfiledController(this.pid, this.feedforward);
        this.veloFilter = veloFilter;
        this.accelFilter = accelFilter;

        derivTimer.reset();

        reset();
    }

    /**
     * @param kG            Additive constant motor power (voltage compensated)
     * @param UNIT_PER_TICK Arbitrary unit per tick scale factor
     */
    public void updateConstants(double kG, double UNIT_PER_TICK) {
        this.kG = kG;
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
        controller.profiler.setTarget(new MotionState(currentPosition, currentVelocity), new MotionState(targetPosition, 0.0));
    }

    /**
     * Resets internal states to 0
     */
    public void reset() {
        accelFilter.clearMemory();
        veloFilter.clearMemory();

        motors[0].encoder.reset();
        pid.resetIntegral();
        pid.derivFilter.clearMemory();

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
        run(controller.calculate(new MotionState(currentPosition, 0.0), currentBatteryVoltage), false);
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
        telemetry.addData("Current position (in)", currentPosition);
        telemetry.addData("Profile position (in)", controller.profiler.getX());
        telemetry.addLine();
        telemetry.addData("Current velocity (in/s)", currentVelocity);
        telemetry.addData("Encoder-calculated velocity (in/s)", motors[0].encoder.getCorrectedVelocity());
        telemetry.addData("Profile velocity (in/s)", controller.profiler.getV());
        telemetry.addData("Max velocity (in/s)", maxVelocity);
        telemetry.addLine();
        telemetry.addData("Current acceleration (in/s^2)", currentAcceleration);
        telemetry.addData("Max acceleration (in/s^2)", maxAcceleration);
        telemetry.addLine();
        telemetry.addData("Error integral (in*s)", pid.getErrorIntegral());
        telemetry.addData("Error (in)", pid.getError());
        telemetry.addData("Error derivative (in/s)", pid.getErrorDerivative());
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
