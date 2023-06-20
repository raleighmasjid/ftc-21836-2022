package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.ProfiledLift;

public class PowerplayLift extends ProfiledLift {

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

        veloFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_VELO, RobotConfig.LIFT_FILTER_COUNT_VELO);
        accelFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_ACCEL, RobotConfig.LIFT_FILTER_COUNT_ACCEL);

        controller.pid.setGains(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL
        );
        controller.pid.derivFilter.setGains(
                RobotConfig.LIFT_FILTER_GAIN_kD,
                RobotConfig.LIFT_FILTER_COUNT_kD
        );
        controller.feedforward.setGains(
                goingDown ? RobotConfig.LIFT_kV_DOWN : RobotConfig.LIFT_kV_UP,
                goingDown ? RobotConfig.LIFT_kA_DOWN : RobotConfig.LIFT_kA_UP,
                goingDown ? RobotConfig.LIFT_kS_DOWN : RobotConfig.LIFT_kS_UP
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
    private double kG() {
        double currentPosition = getCurrentPosition();
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
