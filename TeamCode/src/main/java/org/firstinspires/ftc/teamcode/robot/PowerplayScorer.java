package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.systems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.systems.ProfiledLift;
import org.firstinspires.ftc.teamcode.systems.SimpleClaw;

/**
 * Contains a {@link ProfiledClawArm} and {@link ProfiledLift} linked by automated methods
 *
 * @author Arshad Anas
 * @since 2022/12/24
 */
public class PowerplayScorer {

    private final SimpleServo coneArmServoR;
    private final SimpleServo coneArmServoL;

    private final ElapsedTime liftClawTimer = new ElapsedTime();

    public final ProfiledLift lift;

    public final ProfiledClawArm passthrough;

    private boolean clawHasLifted = true;

    /**
     * Named lift position
     */
    public enum LiftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    private SimpleServo goBILDAServo(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 280);
    }

    private MotorEx liftMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 145.1, 1150);
    }

    private SimpleServo axonMINI(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    /**
     * Initialize fields
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayScorer(HardwareMap hw) {
        MotorEx liftMotor1 = liftMotor(hw, "lift motor 1");
        MotorEx liftMotor2 = liftMotor(hw, "lift motor 2");
        MotorEx liftMotor3 = liftMotor(hw, "lift motor 3");

        liftMotor2.setInverted(false);
        liftMotor1.setInverted(true);
        liftMotor3.setInverted(true);

        lift = new ProfiledLift(
                new MotorGroup(liftMotor2, liftMotor1, liftMotor3),
                hw.voltageSensor.iterator().next(),
                new PIDFController(
                        new PIDController(0, 0, 0, 0, new IIRLowPassFilter(0)),
                        new FeedforwardController(0, 0, 0)
                ),
                new FIRLowPassFilter(0, 0),
                new FIRLowPassFilter(0, 0)
        );

        SimpleServo passThruServoL = axonMINI(hw, "passthrough 2");
        passThruServoL.setInverted(true);

        passthrough = new ProfiledClawArm(
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                new SimpleClaw(axonMINI(hw, "claw right"), RobotConfig.ANGLE_CLAW_OPEN, RobotConfig.ANGLE_CLAW_CLOSED),
                axonMINI(hw, "claw pivot"),
                axonMINI(hw, "passthrough 1"),
                passThruServoL
        );

        updateValues();

        coneArmServoR = goBILDAServo(hw, "arm right");
        coneArmServoL = goBILDAServo(hw, "arm left");
        coneArmServoL.setInverted(true);

        liftClawTimer.reset();
    }

    public double getConesHeight(int numOfCones) {
        return (numOfCones - 1) * (RobotConfig.HEIGHT_2 - RobotConfig.HEIGHT_FLOOR) + RobotConfig.HEIGHT_FLOOR;
    }

    public void setTargetLiftPos(LiftPos height) {
        passthrough.setTilt(height == LiftPos.LOW || height == LiftPos.MED || height == LiftPos.TALL);
        switch (height) {
            case TALL:
                lift.setTargetPosition(RobotConfig.HEIGHT_TALL, "Tall junction");
                break;
            case MED:
                lift.setTargetPosition(RobotConfig.HEIGHT_MEDIUM, "Medium junction");
                break;
            case LOW:
                lift.setTargetPosition(RobotConfig.HEIGHT_LOW, "Low junction");
                break;
            case FIVE:
                lift.setTargetPosition(getConesHeight(5), "5 cones");
                break;
            case FOUR:
                lift.setTargetPosition(getConesHeight(4), "4 cones");
                break;
            case THREE:
                lift.setTargetPosition(getConesHeight(3), "3 cones");
                break;
            case TWO:
                lift.setTargetPosition(getConesHeight(2), "2 cones / ground junction");
                break;
            case FLOOR:
            default:
                lift.setTargetPosition(getConesHeight(1), "Floor / 1 cone");
                break;
        }
    }

    public void updateValues() {
        boolean goingDown = lift.getTargetPosition() < lift.getCurrentPosition();

        lift.veloFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_VELO, RobotConfig.LIFT_FILTER_COUNT_VELO);
        lift.accelFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_ACCEL, RobotConfig.LIFT_FILTER_COUNT_ACCEL);

        lift.controller.pid.setGains(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL
        );
        lift.controller.pid.derivFilter.setGains(RobotConfig.LIFT_FILTER_GAIN_kD);
        lift.controller.feedforward.setGains(
                goingDown ? RobotConfig.LIFT_kV_DOWN : RobotConfig.LIFT_kV_UP,
                goingDown ? RobotConfig.LIFT_kA_DOWN : RobotConfig.LIFT_kA_UP,
                RobotConfig.LIFT_kS
        );
        lift.controller.setOutputBounds(-1.0, 1.0);

        lift.updateGains(
                kG(),
                RobotConfig.LIFT_INCHES_PER_TICK,
                RobotConfig.LIFT_MAX_VELO,
                RobotConfig.LIFT_MAX_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );

        passthrough.updateValues(
                RobotConfig.ANGLE_PASS_FRONT,
                RobotConfig.ANGLE_PASS_BACK,
                RobotConfig.ANGLE_PIVOT_FRONT,
                RobotConfig.ANGLE_PIVOT_BACK,
                RobotConfig.ANGLE_PIVOT_POS,
                RobotConfig.PASS_PIVOT_POS_TOLERANCE,
                RobotConfig.ANGLE_PASS_TILT_OFFSET,
                RobotConfig.ANGLE_PASS_MINI_TILT_OFFSET,
                RobotConfig.PASS_MAX_VELO,
                RobotConfig.PASS_MAX_ACCEL,
                RobotConfig.PASS_MAX_JERK
        );

        passthrough.claw.updateAngles(RobotConfig.ANGLE_CLAW_OPEN, RobotConfig.ANGLE_CLAW_CLOSED);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    private double kG() {
        return lift.getCurrentPosition() >= RobotConfig.HEIGHT_STAGES_4 ? RobotConfig.LIFT_kG_4 :
                lift.getCurrentPosition() >= RobotConfig.HEIGHT_STAGES_3 ? RobotConfig.LIFT_kG_3 :
                        lift.getCurrentPosition() >= RobotConfig.HEIGHT_STAGES_2 ? RobotConfig.LIFT_kG_2 :
                                lift.getCurrentPosition() > RobotConfig.LIFT_TOLERANCE_POS ? RobotConfig.LIFT_kG_1 :
                                        0.0;
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        lift.reset();
        passthrough.setTilt(false);
    }

    /**
     * Runs {@link #grabCone} if open
     * Runs {@link #dropCone} if already closed
     */
    public void triggerClaw() {
        if (!passthrough.claw.getClosed()) grabCone();
        else dropCone();
    }

    /**
     * Closes claw
     * Waits for claw to close
     * Runs {@link #liftClaw}
     */
    public void grabCone() {
        passthrough.claw.setClosed(true);
        if (lift.getCurrentPosition() <= (getConesHeight(5) + RobotConfig.LIFT_TOLERANCE_POS)) {
            clawHasLifted = false;
            liftClawTimer.reset();
        }
    }

    /**
     * Lifts claw either:
     * 6 inches if grabbing off a stack
     * 2 inches if grabbing off the floor
     */
    public void liftClaw() {
        lift.setTargetPosition(lift.getCurrentPosition() + ((lift.getCurrentPosition() > RobotConfig.LIFT_TOLERANCE_POS) ? 6 : 2));
        clawHasLifted = true;
    }

    /**
     * Opens claw and runs lift to floor position
     */
    public void dropCone() {
        dropCone(LiftPos.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     *
     * @param height Named position to run lift to
     */
    public void dropCone(LiftPos height) {
        passthrough.claw.setClosed(false);
        setTargetLiftPos(height);
    }

    /**
     * Holds {@link #coneArmServoR} and {@link #coneArmServoL} positions
     *
     * @param angleR The angle to turn {@link #coneArmServoR} to
     * @param angleL The angle to turn {@link #coneArmServoL} to
     */
    public void run(double angleR, double angleL) {
        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
        coneArmServoL.turnToAngle(angleL);
        coneArmServoR.turnToAngle(angleR);
    }

    /**
     * Print tuning telemetry from {@link #lift} and {@link #passthrough}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        passthrough.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        lift.printNumericalTelemetry(telemetry);
    }

    /**
     * Print lift, claw, pivot, and passthrough statuses
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        lift.printTelemetry(telemetry);
        telemetry.addLine();
        passthrough.printTelemetry(telemetry);
    }
}
