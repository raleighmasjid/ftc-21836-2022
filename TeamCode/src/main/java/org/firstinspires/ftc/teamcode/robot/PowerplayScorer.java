package org.firstinspires.ftc.teamcode.robot;


import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Merger class, linking a {@link PowerplayPassthrough} and {@link PowerplayLift} by automated methods
 *
 * @author Arshad Anas
 * @since 2022/12/24
 */
public class PowerplayScorer {

    protected final SimpleServo coneArmServoR;
    protected final SimpleServo coneArmServoL;

    protected final ElapsedTime liftClawTimer = new ElapsedTime();

    public final PowerplayLift lift;

    public final PowerplayPassthrough passthrough;

    protected boolean clawHasLifted = true;

    public static SimpleServo getGoBILDAServo(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 280);
    }

    public static SimpleServo getAxon(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    public static SimpleServo getReversedServo(SimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public static MotorEx getLiftMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 145.1, 1150);
    }

    public static MotorEx[] getLiftMotors(HardwareMap hw) {

        MotorEx liftMotor1 = getLiftMotor(hw, "lift motor 1");
        MotorEx liftMotor2 = getLiftMotor(hw, "lift motor 2");
        MotorEx liftMotor3 = getLiftMotor(hw, "lift motor 3");

        liftMotor2.setInverted(false);
        liftMotor1.setInverted(true);
        liftMotor3.setInverted(true);

        return new MotorEx[]{liftMotor2, liftMotor1, liftMotor3};
    }

    /**
     * Initialize fields
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayScorer(HardwareMap hw) {

        lift = new PowerplayLift(hw);
        passthrough = new PowerplayPassthrough(hw);

        coneArmServoR = getGoBILDAServo(hw, "arm right");
        coneArmServoL = getReversedServo(getGoBILDAServo(hw, "arm left"));

        reset();
    }

    public void reset() {
        lift.reset();
        passthrough.reset();
        clawHasLifted = true;
        liftClawTimer.reset();
    }

    public void setTargetLiftPos(PowerplayLift.Position height) {
        passthrough.setTilt(height == PowerplayLift.Position.LOW || height == PowerplayLift.Position.MED || height == PowerplayLift.Position.TALL);
        lift.setTargetPosition(height);
    }

    /**
     * Runs {@link #grabCone} if open <p>
     * Runs {@link #dropCone} if already closed
     */
    public void triggerClaw() {
        if (!passthrough.claw.getClosed()) grabCone();
        else dropCone();
    }

    /**
     * Closes claw <p>
     * Waits for claw to close <p>
     * Runs {@link #liftClaw}
     */
    public void grabCone() {
        passthrough.claw.setClosed(true);
        if (lift.getCurrentPosition() <= (lift.getConesHeight(5) + RobotConfig.LIFT_TOLERANCE_POS)) {
            clawHasLifted = false;
            liftClawTimer.reset();
        }
    }

    /**
     * Lifts claw either: <p>
     * 6 inches if grabbing off a stack <p>
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
        dropCone(PowerplayLift.Position.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     *
     * @param height Named position to run lift to
     */
    public void dropCone(PowerplayLift.Position height) {
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
        passthrough.run();
        passthrough.runPivot();
        passthrough.claw.run();
        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
        coneArmServoL.turnToAngle(angleL);
        coneArmServoR.turnToAngle(angleR);
    }

}
