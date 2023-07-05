package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.ProfiledPIDF;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
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
            kG_4 = 0.25,
            kG_3 = 0.21,
            kP = 0.25,
            kI = 0.2,
            kD = 0.01,
            kV_DOWN = 0.006,
            kV_UP = 0.005,
            kA_DOWN = 0.0015,
            kA_UP = 0.00075,
            kS = 0.17,
            FILTER_GAIN_kD = 0.875,
            FILTER_GAIN_VELO = 0.8,
            FILTER_GAIN_ACCEL = 0.8,
            MAX_VELO = 46.6,
            MAX_ACCEL = 189.16,
            MAX_JERK = 600.0,
            MAX_PID_OUTPUT_WITH_INTEGRAL = 0.6,
            TOLERANCE = 0.15843625,
            INCHES_PER_TICK = 0.03168725;

    public static int
            FILTER_COUNT_kD = 300,
            FILTER_COUNT_VELO = 300,
            FILTER_COUNT_ACCEL = 300;

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
     */
    public PowerplayLift(HardwareMap hw) {
        super(getLiftMotors(hw), hw.voltageSensor.iterator().next(), new ProfiledPIDF(), new FIRLowPassFilter(), new FIRLowPassFilter());
        updateConstants();
        controller.setOutputBounds(-1.0, 1.0);
    }

    @Override
    public void readPosition() {
        updateConstants();
        super.readPosition();
    }

    private void updateConstants() {
        boolean goingDown = getTargetPosition() < getCurrentPosition();

        veloFilter.setGains(FILTER_GAIN_VELO, FILTER_COUNT_VELO);
        accelFilter.setGains(FILTER_GAIN_ACCEL, FILTER_COUNT_ACCEL);

        controller.pid.setGains(
                kP,
                kI,
                kD,
                MAX_PID_OUTPUT_WITH_INTEGRAL
        );
        controller.pid.derivFilter.setGains(
                FILTER_GAIN_kD,
                FILTER_COUNT_kD
        );
        controller.feedforward.setGains(
                goingDown ? kV_DOWN : kV_UP,
                goingDown ? kA_DOWN : kA_UP,
                kS
        );
        controller.updateConstraints(
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );

        super.updateConstants(kG(), INCHES_PER_TICK);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    private double kG() {
        double currentPosition = getCurrentPosition();
        return currentPosition >= HEIGHT_1_STAGE * 3 ? kG_4 :
                currentPosition >= HEIGHT_1_STAGE * 2 ? kG_3 :
                        currentPosition >= HEIGHT_1_STAGE ? kG_3 - (kG_4 - kG_3) :
                                currentPosition > TOLERANCE ? kG_3 - 2 * (kG_4 - kG_3) :
                                        0.0;
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
}
