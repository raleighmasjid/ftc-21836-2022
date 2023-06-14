package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Contains a {@link PowerplayPassthrough} and {@link PowerplayLift} linked by automated methods
 *
 * @author Arshad Anas
 * @since 2022/12/24
 */
public class PowerplayScorer {

    private SimpleServo coneArmServoR, coneArmServoL;

    private ElapsedTime liftClawTimer = new ElapsedTime();

    public PowerplayLift lift;

    public PowerplayPassthrough passthrough;

    private boolean clawHasLifted = true;

    private SimpleServo goBILDAServo(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 280);
    }

    /**
     * Initialize fields
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayScorer(HardwareMap hw) {
        coneArmServoR = goBILDAServo(hw, "arm right");
        coneArmServoL = goBILDAServo(hw, "arm left");
        lift = new PowerplayLift(hw);
        passthrough = new PowerplayPassthrough(hw);
        liftClawTimer.reset();
    }

    public void setTargetLiftPos(PowerplayLift.Position height) {
        passthrough.setTilt(height == PowerplayLift.Position.LOW || height == PowerplayLift.Position.MED || height == PowerplayLift.Position.TALL);
        lift.setTargetPosition(height);
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        lift.resetLift();
        passthrough.setTilt(false);
    }

    /**
     * Runs {@link #grabCone} if open
     * Runs {@link #dropCone} if already closed
     */
    public void triggerClaw() {
        if (passthrough.getClawIsOpen()) grabCone();
        else dropCone();
    }

    /**
     * Closes claw
     * Waits for claw to close
     * Runs {@link #liftClaw}
     */
    public void grabCone() {
        passthrough.setClawOpen(false);
        if (lift.getCurrentPosition() <= (lift.getConesHeight(5) + RobotConfig.LIFT_TOLERANCE_POS)) {
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
        dropCone(PowerplayLift.Position.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     *
     * @param height Named position to run lift to
     */
    public void dropCone(PowerplayLift.Position height) {
        passthrough.setClawOpen(true);
        setTargetLiftPos(height);
    }

    /**
     * Lifts claw, if previously triggered
     */
    public void runLiftClaw() {
        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
    }

    /**
     * Holds {@link #coneArmServoR} and {@link #coneArmServoL} positions
     *
     * @param angleR The angle to turn {@link #coneArmServoR} to
     * @param angleL The angle to turn {@link #coneArmServoL} to
     */
    public void runConeArms(double angleR, double angleL) {
        coneArmServoL.turnToAngle(280.0 - Math.min(angleL, 110.0));
        coneArmServoR.turnToAngle(Math.min(angleR, 110.0));
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
