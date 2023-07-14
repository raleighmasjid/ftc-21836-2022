package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.fullstatefeedback.ProfiledFullStateVA;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FullStateGains;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.controllers.pid.ProfiledPIDVA;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.ProfiledMotor;

@Config
public class Lift extends ProfiledMotor {

    public static double
            HEIGHT_FLOOR = 0.0,
            HEIGHT_2_CONES = 1.35,
            HEIGHT_LOW = 6.6,
            HEIGHT_MEDIUM = 17,
            HEIGHT_TALL = 27,
            HEIGHT_1_STAGE = 9.6,
            kG_1_STAGE = 0.06,
            kG_3 = 0.312,
            kI = 0.2,
            kV = 0.0155,
            kA = 0.0025,
            kStatic = 0.035,
            pGain = 0.0,
            vGain = 0.0,
            aGain = 0.0,
            FILTER_GAIN_VELO = 0.5,
            FILTER_GAIN_ACCEL = 0.5,
            MAX_VELO = 32,
            MAX_ACCEL = 189.16,
            MAX_JERK = 600.0,
            MAX_PID_OUTPUT_WITH_INTEGRAL = 0.6,
            TOLERANCE = 0.15843625,
            INCHES_PER_TICK = 0.03168725;

    public static int
            FILTER_COUNT_VELO = 10,
            FILTER_COUNT_ACCEL = 10;

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

    public Lift(HardwareMap hw) {
        super(getLiftMotors(hw), hw.voltageSensor.iterator().next(), new ProfiledPIDVA(), new ProfiledFullStateVA(), new FIRLowPassFilter(), new FIRLowPassFilter());
    }

    @Override
    public void readPosition() {
        veloFilter.setGains(FILTER_GAIN_VELO, FILTER_COUNT_VELO);
        accelFilter.setGains(FILTER_GAIN_ACCEL, FILTER_COUNT_ACCEL);

        pid.setGains(
                new PIDGains(0.0, kI, 0.0, MAX_PID_OUTPUT_WITH_INTEGRAL),
                new FeedforwardGains(0, 0, 0)
        );
        fullState.setGains(
                new FullStateGains(pGain, vGain, aGain),
                new FeedforwardGains(kV, kA, kStatic)
        );
        pid.updateConstraints(
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );
        fullState.updateConstraints(
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );

        super.updateScale(INCHES_PER_TICK);
        super.readPosition();
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
     * Run motor(s) at the entered percentage of max velocity
     *
     * @param veloCommand       Pass in a velocity command between -1 ≤ x ≤ 1
     * @param voltageCompensate Whether to voltage compensate veloCommand
     */
    @Override
    public void run(double veloCommand, boolean voltageCompensate) {
        veloCommand += kG() * (voltageCompensate ? 1.0 : 12.0 / currentBatteryVoltage);
        super.run(veloCommand, voltageCompensate);
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
}
