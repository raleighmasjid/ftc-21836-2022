package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.ProfiledLift;

@Config
public class PowerplayLift extends ProfiledLift {

    public static double
            HEIGHT_FLOOR = 0.0,
            HEIGHT_2_CONES = 1.35,
            HEIGHT_LOW = 6.2,
            HEIGHT_MEDIUM = 16.4,
            HEIGHT_TALL = 26.6,
            HEIGHT_1_STAGE = 9.6,
            LIFT_kG_4 = 0.25,
            LIFT_kG_3 = 0.21,
            LIFT_kP = 0.25,
            LIFT_kI = 0.2,
            LIFT_kD = 0.01,
            LIFT_kV_DOWN = 0.006,
            LIFT_kV_UP = 0.005,
            LIFT_kA_DOWN = 0.0015,
            LIFT_kA_UP = 0.00075,
            LIFT_kS = 0.17,
            LIFT_FILTER_GAIN_kD = 0.875,
            LIFT_FILTER_GAIN_VELO = 0.8,
            LIFT_FILTER_GAIN_ACCEL = 0.8,
            LIFT_MAX_VELO = 46.6,
            LIFT_MAX_ACCEL = 189.16,
            LIFT_MAX_JERK = 600.0,
            LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL = 0.6,
            LIFT_TOLERANCE_POS = 0.15843625,
            LIFT_INCHES_PER_TICK = 0.03168725;

    public static int
            LIFT_FILTER_COUNT_kD = 300,
            LIFT_FILTER_COUNT_VELO = 300,
            LIFT_FILTER_COUNT_ACCEL = 300;

    /**
     * Named lift position
     */
    public enum Position {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    public static MotorEx liftMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 145.1, 1150);
    }

    public static MotorEx[] getLiftMotors(HardwareMap hw) {

        MotorEx[] motors = {
                liftMotor(hw, "lift motor 2"),
                liftMotor(hw, "lift motor 1"),
                liftMotor(hw, "lift motor 3")
        };

        motors[1].setInverted(true);
        motors[2].setInverted(true);

        return motors;
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update constants
     */
    public PowerplayLift(HardwareMap hw) {
        super(getLiftMotors(hw), hw.voltageSensor.iterator().next(), new PIDFController(), new FIRLowPassFilter(), new FIRLowPassFilter());
        updateConstants();
    }

    @Override
    public void readPosition() {
        updateConstants();
        super.readPosition();
    }

    private void updateConstants() {
        boolean goingDown = getTargetPosition() < getCurrentPosition();

        veloFilter.setGains(LIFT_FILTER_GAIN_VELO, LIFT_FILTER_COUNT_VELO);
        accelFilter.setGains(LIFT_FILTER_GAIN_ACCEL, LIFT_FILTER_COUNT_ACCEL);

        controller.pid.setGains(
                LIFT_kP,
                LIFT_kI,
                LIFT_kD,
                LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL
        );
        controller.pid.derivFilter.setGains(
                LIFT_FILTER_GAIN_kD,
                LIFT_FILTER_COUNT_kD
        );
        controller.feedforward.setGains(
                goingDown ? LIFT_kV_DOWN : LIFT_kV_UP,
                goingDown ? LIFT_kA_DOWN : LIFT_kA_UP,
                LIFT_kS
        );
        controller.setOutputBounds(-1.0, 1.0);

        updateConstants(
                kG(),
                LIFT_INCHES_PER_TICK,
                LIFT_MAX_VELO,
                LIFT_MAX_ACCEL,
                LIFT_MAX_JERK
        );
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    private double kG() {
        double currentPosition = getCurrentPosition();
        return currentPosition >= HEIGHT_1_STAGE * 3 ? LIFT_kG_4 :
                currentPosition >= HEIGHT_1_STAGE * 2 ? LIFT_kG_3 :
                        currentPosition >= HEIGHT_1_STAGE ? LIFT_kG_3 - (LIFT_kG_4 - LIFT_kG_3) :
                                currentPosition > LIFT_TOLERANCE_POS ? LIFT_kG_3 - 2 * (LIFT_kG_4 - LIFT_kG_3) :
                                        0.0;
    }

    public double getConesHeight(int numOfCones) {
        return (numOfCones - 1) * (HEIGHT_2_CONES - HEIGHT_FLOOR) + HEIGHT_FLOOR;
    }

    public void setTargetPosition(PowerplayLift.Position height) {
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
}
