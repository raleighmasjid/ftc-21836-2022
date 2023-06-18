package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.systems.ProfiledLift;

public class PowerplayLift extends ProfiledLift {

    /**
     * Named lift position
     */
    public enum Position {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update constants
     */
    public PowerplayLift(
            MotorEx[] motors,
            VoltageSensor batteryVoltageSensor,
            PIDFController controller,
            FIRLowPassFilter veloFilter,
            FIRLowPassFilter accelFilter
    ) {
        super(motors, batteryVoltageSensor, controller, veloFilter, accelFilter);
        updateConstants();
    }

    @Override
    public void readPosition() {
        updateConstants();
        super.readPosition();
    }

    protected void updateConstants() {
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
        controller.setOutputBounds(-1.0, 1.0);

        updateConstants(
                kG(),
                RobotConfig.LIFT_INCHES_PER_TICK,
                RobotConfig.LIFT_MAX_VELO,
                RobotConfig.LIFT_MAX_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    protected double kG() {
        return currentPosition >= RobotConfig.HEIGHT_1_STAGE * 3 ? RobotConfig.LIFT_kG_4 :
                currentPosition >= RobotConfig.HEIGHT_1_STAGE * 2 ? RobotConfig.LIFT_kG_3 :
                        currentPosition >= RobotConfig.HEIGHT_1_STAGE ? RobotConfig.LIFT_kG_3 - (RobotConfig.LIFT_kG_4 - RobotConfig.LIFT_kG_3) :
                                currentPosition > RobotConfig.LIFT_TOLERANCE_POS ? RobotConfig.LIFT_kG_3 - 2 * (RobotConfig.LIFT_kG_4 - RobotConfig.LIFT_kG_3) :
                                        0.0;
    }

    public double getConesHeight(int numOfCones) {
        return (numOfCones - 1) * (RobotConfig.HEIGHT_2_CONES - RobotConfig.HEIGHT_FLOOR) + RobotConfig.HEIGHT_FLOOR;
    }

    public void setTargetPosition(PowerplayLift.Position height) {
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
}
