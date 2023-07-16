package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.Differentiator;
import org.firstinspires.ftc.teamcode.control.MotionProfiler;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controllers.FullStateController;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FullStateGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ProfileConstraints;

@Config
public class Lift {

    public static double
            HEIGHT_FLOOR = 0.0,
            HEIGHT_2_CONES = 1.35,
            HEIGHT_LOW = 6.6,
            HEIGHT_MEDIUM = 17,
            HEIGHT_TALL = 27,
            HEIGHT_1_STAGE = 9.6,
            kG_1_STAGE = 0.06,
            kG_3 = 0.312,
            FILTER_GAIN_VELO = 0.5,
            FILTER_GAIN_ACCEL = 0.8,
            TOLERANCE = 0.15843625,
            INCHES_PER_TICK = 0.03168725;

    public static int
            FILTER_COUNT_VELO = 20,
            FILTER_COUNT_ACCEL = 50;

    public static FullStateGains fullStateGains = new FullStateGains(
            0.075,
            0.0125,
            0.0
    );

    public static PIDGains integratorGains = new PIDGains(
            0.0,
            0.3,
            0.0,
            0.6
    );

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.0075,
            0.0005,
            0.035
    );

    public static ProfileConstraints constraints = new ProfileConstraints(
            32.0,
            189.16,
            450.0
    );

    private final Differentiator veloCalculator = new Differentiator();
    private final Differentiator accelCalculator = new Differentiator();
    private final PIDController integrator = new PIDController();
    private final FullStateController fullState = new FullStateController();
    private final FeedforwardController feedforward = new FeedforwardController();
    private final MotionProfiler profiler = new MotionProfiler();
    private final MotorEx[] motors;
    private final VoltageSensor batteryVoltageSensor;
    private final HardwareMap hardwareMap;

    private String targetPositionName = "Zero";
    private double targetPosition, maxVelocity, maxAcceleration, currentBatteryVoltage = 12.0;
    private State currentState = new State();

    public double getCurrentPosition() {
        return currentState.x;
    }

    /**
     * Named lift position
     */
    public enum Position {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    public MotorEx liftMotor(String name) {
        return new MotorEx(hardwareMap, name, 145.1, 1150);
    }

    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motors = new MotorEx[]{
                liftMotor("lift motor 2"),
                liftMotor("lift motor 1"),
                liftMotor("lift motor 3")
        };

        motors[1].setInverted(true);
        motors[2].setInverted(true);

        for (MotorEx motor : this.motors) motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        reset();
    }

    public void readPosition() {
        accelCalculator.filter.setGains(FILTER_GAIN_VELO, FILTER_COUNT_VELO);
        veloCalculator.filter.setGains(FILTER_GAIN_ACCEL, FILTER_COUNT_ACCEL);

        fullState.setGains(fullStateGains);
        integrator.setGains(integratorGains);
        feedforward.setGains(feedforwardGains);
        profiler.updateConstraints(constraints);

        currentBatteryVoltage = batteryVoltageSensor.getVoltage();

        double x = motors[0].encoder.getPosition() * INCHES_PER_TICK;
        double v = veloCalculator.getDerivative(x);
        double a = accelCalculator.getDerivative(v);

        currentState = new State(x, v, a);

        maxVelocity = Math.max(v, maxVelocity);
        maxAcceleration = Math.max(a, maxAcceleration);
    }

    public double getConesHeight(int numOfCones) {
        return (numOfCones - 1) * (HEIGHT_2_CONES - HEIGHT_FLOOR) + HEIGHT_FLOOR;
    }

    public void setTargetPosition(Position height) {
        switch (height) {
            case TALL:
                setTargetPosition(HEIGHT_TALL, "Tall junction");
                break;
            case MED:
                setTargetPosition(HEIGHT_MEDIUM, "Medium junction");
                break;
            case LOW:
                setTargetPosition(HEIGHT_LOW, "Low junction");
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
     * Set target for motion profile
     *
     * @param targetPosition Desired position (in inches) to run to
     */
    public void setTargetPosition(double targetPosition) {
        setTargetPosition(targetPosition, Double.toString(targetPosition));
    }

    /**
     * Runs {@link #integrator}
     */
    public void runToPosition() {
        profiler.update();
        State setpoint = new State(profiler.getX(), profiler.getV(), profiler.getA());
        fullState.setTarget(setpoint);
        integrator.setTarget(setpoint);
        feedforward.setTarget(setpoint);

        double fullStateOutput = fullState.calculate(currentState);
        double integratorOutput = integrator.calculate(currentState);
        double feedforwardOutput = feedforward.calculate(currentBatteryVoltage, fullStateOutput);

        run(fullStateOutput + feedforwardOutput + integratorOutput, false);
    }

    /**
     * Run motor(s) at the entered percentage of max velocity
     *
     * @param veloCommand       Pass in a velocity command between -1 ≤ x ≤ 1
     * @param voltageCompensate Whether to voltage compensate veloCommand
     */
    public void run(double veloCommand, boolean voltageCompensate) {
        double scalar = 12.0 / currentBatteryVoltage;
        if (voltageCompensate) veloCommand *= scalar;
        for (MotorEx motor : motors) motor.set(kG() * scalar + veloCommand);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    private double kG() {
        double currentPosition = getCurrentPosition();
        if (currentPosition >= HEIGHT_1_STAGE * 3) return kG_3 + kG_1_STAGE;
        if (currentPosition >= HEIGHT_1_STAGE * 2) return kG_3;
        if (currentPosition >= HEIGHT_1_STAGE) return kG_3 - kG_1_STAGE;
        if (currentPosition > TOLERANCE) return kG_3 - 2 * kG_1_STAGE;
        return 0.0;
    }

    /**
     * Resets internal states to 0
     */
    public void reset() {
        accelCalculator.filter.reset();
        veloCalculator.filter.reset();

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
     * Print tuning telemetry from {@link #readPosition} and {@link #integrator}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current position (in)", currentState.x);
        telemetry.addData("Profile position (in)", profiler.getX());
        telemetry.addLine();
        telemetry.addData("Current velocity (in/s)", currentState.v);
        telemetry.addData("Profile velocity (in/s)", profiler.getV());
        telemetry.addData("Max velocity (in/s)", maxVelocity);
        telemetry.addLine();
        telemetry.addData("Current acceleration (in/s^2)", currentState.a);
        telemetry.addData("Profile acceleration (in/s^2)", profiler.getA());
        telemetry.addData("Max acceleration (in/s^2)", maxAcceleration);
        telemetry.addLine();
        telemetry.addData("Position error integral (in*s)", integrator.getErrorIntegral());
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
