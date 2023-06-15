package org.firstinspires.ftc.teamcode.systems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;

/**
 * Contains a 3-motor motion-profiled lift
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class ProfiledLift {
    /**
     * Motor powering the dual lift system
     */
    private final MotorGroup motors;

    private final ElapsedTime profileTimer = new ElapsedTime();
    private final ElapsedTime derivTimer = new ElapsedTime();

    public final FIRLowPassFilter accelFilter;
    public final FIRLowPassFilter veloFilter;

    private final VoltageSensor batteryVoltageSensor;

    /**
     * PIDF controller for lift
     */
    public final PIDFController controller;

    private MotionProfile profile;

    /**
     * Immediate target lift state grabbed from {@link #profile}
     */
    private MotionState profileState = new MotionState(0.0, 0.0, 0.0, 0.0);

    private String targetPositionName = "Zero";

    private double currentBatteryVoltage = 12.0;
    private double currentPosition = 0.0;
    private double currentVelocity = 0.0;
    private double currentAcceleration = 0.0;
    private double targetPosition = 0.0;
    private double maxVelocity = 0.0;
    private double maxAcceleration = 0.0;
    private double kG = 0.0;
    private double inchesPerTick = 0;
    private double maxProfileVelo, maxProfileAccel, maxProfileJerk;

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Initialize fields
     */
    public ProfiledLift(
            MotorGroup motors,
            VoltageSensor batteryVoltageSensor,
            PIDFController controller,
            FIRLowPassFilter veloFilter,
            FIRLowPassFilter accelFilter
    ) {
        this.motors = motors;
        this.motors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.batteryVoltageSensor = batteryVoltageSensor;

        profileTimer.reset();
        derivTimer.reset();

        this.veloFilter = veloFilter;
        this.accelFilter = accelFilter;
        this.controller = controller;

        updateProfile();
    }

    /**
     * Update {@link #controller}, {@link #veloFilter}, and {@link #accelFilter} gains
     */
    public void updateGains(
            double kG,
            double inchesPerTick,
            double maxProfileVelo,
            double maxProfileAccel,
            double maxProfileJerk
    ) {
        this.kG = kG;
        this.inchesPerTick = inchesPerTick;
        this.maxProfileVelo = maxProfileVelo;
        this.maxProfileAccel = maxProfileAccel;
        this.maxProfileJerk = maxProfileJerk;
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
        currentPosition = motors.encoder.getPosition() * inchesPerTick;
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
    public void setTargetPosition(double targetPosition, String targetPositionName) {
        this.targetPosition = targetPosition;
        this.targetPositionName = targetPositionName;
        updateProfile();
    }

    /**
     * Set target for lift motion profile
     *
     * @param targetPosition Desired position (in inches) to run to
     */
    public void setTargetPosition(double targetPosition) {
        setTargetPosition(targetPosition, "Custom position");
    }

    /**
     * Update {@link #profile} with a {@link #targetPosition} set with {@link #setTargetPosition}
     */
    private void updateProfile() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPosition, currentVelocity),
                new MotionState(targetPosition, 0.0),
                maxProfileVelo,
                maxProfileAccel,
                maxProfileJerk
        );
        profileTimer.reset();
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        accelFilter.clearMemory();
        veloFilter.clearMemory();

        motors.resetEncoder();
        controller.pid.resetIntegral();
        controller.pid.derivFilter.clearMemory();

        currentPosition = 0.0;
        currentVelocity = 0.0;
        currentAcceleration = 0.0;
        maxVelocity = 0.0;
        maxAcceleration = 0.0;

        targetPosition = 0.0;
        targetPositionName = "Zero";

        updateProfile();
        profileState = new MotionState(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Runs {@link #controller} to track along {@link #profile}
     */
    public void runToPosition() {
        profileState = profile.get(profileTimer.seconds());

        controller.pid.setTarget(profileState.getX());
        controller.feedforward.setTargetVelocity(profileState.getV());
        controller.feedforward.setTargetAcceleration(profileState.getA());

        if (targetPosition == currentPosition || Math.signum(controller.pid.getError()) != Math.signum(controller.pid.getLastError())) {
            controller.pid.resetIntegral();
        }

        run(controller.update(currentPosition, currentBatteryVoltage), false);
    }

    /**
     * Run {@link #motors} at the entered percentage of max velocity
     *
     * @param veloCommand       Pass in a velocity command between -1 ≤ x ≤ 1
     * @param voltageCompensate Whether to voltage compensate veloCommand
     */
    public void run(double veloCommand, boolean voltageCompensate) {
        double scalar = (12.0 / currentBatteryVoltage);
        if (voltageCompensate) veloCommand *= scalar;
        veloCommand += kG * scalar;
        motors.set(veloCommand);
    }

    /**
     * Print tuning telemetry from {@link #readPosition}, {@link #profile}, and {@link #controller}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current battery voltage", currentBatteryVoltage);
        telemetry.addLine();
        telemetry.addData("Lift current position (in)", currentPosition);
        telemetry.addData("Lift profile position (in)", profileState.getX());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentVelocity);
        telemetry.addData("Lift profile velocity (in/s)", profileState.getV());
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
