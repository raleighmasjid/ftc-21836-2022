package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter;

/**
 * Contains a 3-motor motion-profiled lift
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class PowerplayLift {
    /**
     * Motor powering the dual lift system
     */
    private MotorEx motor1, motor2, motor3;

    private ElapsedTime profileTimer = new ElapsedTime();
    private ElapsedTime derivTimer = new ElapsedTime();

    private FIRLowPassFilter accelFilter;
    private FIRLowPassFilter veloFilter;

    private VoltageSensor batteryVoltageSensor;

    /**
     * PIDF controller for lift
     */
    private PIDFController controller;

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

    /**
     * Named lift position
     */
    public enum Position {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    private MotorEx liftMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 145.1, 1150);
    }

    /**
     * Initialize fields
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayLift(HardwareMap hw) {

        motor1 = liftMotor(hw, "lift motor 1");
        motor2 = liftMotor(hw, "lift motor 2");
        motor3 = liftMotor(hw, "lift motor 3");

        motor1.setInverted(true);
        motor2.setInverted(false);
        motor3.setInverted(true);

        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        controller.setOutputBounds(-1.0, 1.0);

        batteryVoltageSensor = hw.voltageSensor.iterator().next();

        accelFilter = new FIRLowPassFilter(RobotConfig.LIFT_FILTER_GAIN_ACCEL, RobotConfig.LIFT_FILTER_COUNT_ACCEL);
        veloFilter = new FIRLowPassFilter(RobotConfig.LIFT_FILTER_GAIN_VELO, RobotConfig.LIFT_FILTER_COUNT_VELO);

        controller = new PIDFController(
                new PIDController(
                        RobotConfig.LIFT_kP,
                        RobotConfig.LIFT_kI,
                        RobotConfig.LIFT_kD,
                        RobotConfig.LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL,
                        new IIRLowPassFilter(RobotConfig.LIFT_FILTER_GAIN_kD)),
                new FeedforwardController(
                        RobotConfig.LIFT_kV_UP,
                        RobotConfig.LIFT_kA_UP,
                        RobotConfig.LIFT_kS)
        );

        profileTimer.reset();
        derivTimer.reset();

        updateProfile();
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
        updateGains();

        currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        currentPosition = motor2.encoder.getPosition() * RobotConfig.LIFT_INCHES_PER_TICK;
        currentVelocity = veloFilter.getEstimate((currentPosition - lastPosition) / dt);
        currentAcceleration = accelFilter.getEstimate((currentVelocity - lastVelocity) / dt);
        maxVelocity = Math.max(currentVelocity, maxVelocity);
        maxAcceleration = Math.max(currentAcceleration, maxAcceleration);
    }

    /**
     * Update {@link #controller} gains with constants from {@link RobotConfig}
     */
    private void updateGains() {
        boolean goingDown = targetPosition < currentPosition;

        veloFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_VELO, RobotConfig.LIFT_FILTER_COUNT_VELO);
        accelFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_ACCEL, RobotConfig.LIFT_FILTER_COUNT_ACCEL);

        controller.pid.setGains(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL
        );
        controller.pid.derivFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_kD);
        controller.feedforward.setGains(
                goingDown ? RobotConfig.LIFT_kV_DOWN : RobotConfig.LIFT_kV_UP,
                goingDown ? RobotConfig.LIFT_kA_DOWN : RobotConfig.LIFT_kA_UP,
                RobotConfig.LIFT_kS
        );
    }

    public double getConesHeight(int numOfCones) {
        return (numOfCones - 1) * (RobotConfig.HEIGHT_2 - RobotConfig.HEIGHT_FLOOR) + RobotConfig.HEIGHT_FLOOR;
    }

    public void setTargetPosition(Position height) {
        switch (height) {
            case TALL:
                setTargetPosition(RobotConfig.HEIGHT_TALL, "Tall junction");
                break;
            case MED:
                setTargetPosition(RobotConfig.HEIGHT_MEDIUM, "Medium junction");
                break;
            case LOW:
                setTargetPosition(RobotConfig.HEIGHT_LOW, "Low junction");
                break;
            case FIVE:
                setTargetPosition(getConesHeight(5), "5 cones");
                break;
            case FOUR:
                setTargetPosition(getConesHeight(4), "4 cones");
                break;
            case THREE:
                setTargetPosition(getConesHeight(3), "3 cones");
                break;
            case TWO:
                setTargetPosition(getConesHeight(2), "2 cones / ground junction");
                break;
            case FLOOR:
            default:
                setTargetPosition(getConesHeight(1), "Floor / 1 cone");
                break;
        }
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
                RobotConfig.LIFT_MAX_VELO,
                RobotConfig.LIFT_MAX_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );
        profileTimer.reset();
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        accelFilter.clearMemory();
        veloFilter.clearMemory();

        motor2.resetEncoder();
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
    public void runToPos() {
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
     * Run {@link #motor1}, {@link #motor2}, and {@link #motor3} at the entered percentage of max velocity
     *
     * @param veloCommand       Pass in a velocity command between -1 ≤ x ≤ 1
     * @param voltageCompensate Whether to voltage compensate veloCommand
     */
    public void run(double veloCommand, boolean voltageCompensate) {
        double scalar = (12.0 / currentBatteryVoltage);
        if (voltageCompensate) veloCommand *= scalar;
        veloCommand += kG() * scalar;
        motor1.set(veloCommand);
        motor2.set(veloCommand);
        motor3.set(veloCommand);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    private double kG() {
        return currentPosition >= RobotConfig.HEIGHT_STAGES_4 ? RobotConfig.LIFT_kG_4 :
                currentPosition >= RobotConfig.HEIGHT_STAGES_3 ? RobotConfig.LIFT_kG_3 :
                        currentPosition >= RobotConfig.HEIGHT_STAGES_2 ? RobotConfig.LIFT_kG_2 :
                                currentPosition > RobotConfig.LIFT_TOLERANCE_POS ? RobotConfig.LIFT_kG_1 :
                                        0.0;
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
     * Print lift, claw, pivot, and passthrough statuses
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Named target lift position", targetPositionName);
    }
}
