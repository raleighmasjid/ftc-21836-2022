package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Merger class, linking a {@link PowerplayPassthrough} and {@link PowerplayLift} by automated methods
 *
 * @author Arshad Anas
 * @since 2022/12/24
 */
@Config
public class PowerplayScorer {

    private final SimpleServo coneArmR;
    private final SimpleServo coneArmL;

    private final ElapsedTime liftClawTimer = new ElapsedTime();

    public final PowerplayLift lift;

    public final PowerplayPassthrough passthrough;

    private boolean clawHasLifted = true;

    public static double ANGLE_CONE_ARMS_DOWN = 100.0;
    public static double TIME_CLAW = 0.0;

    public static SimpleServo goBILDAServo(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 280);
    }

    /**
     * Initialize fields
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayScorer(HardwareMap hw) {

        lift = new PowerplayLift(hw);
        passthrough = new PowerplayPassthrough(hw);

        coneArmR = goBILDAServo(hw, "arm right");
        coneArmL = goBILDAServo(hw, "arm left");
        coneArmL.setInverted(true);

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
        if (!passthrough.claw.getActivated()) grabCone();
        else dropCone();
    }

    /**
     * Closes claw <p>
     * Waits for claw to close <p>
     * Runs {@link #liftClaw}
     */
    public void grabCone() {
        passthrough.claw.setActivated(true);
        if (lift.getCurrentPosition() <= (lift.getConesHeight(5) + PowerplayLift.TOLERANCE)) {
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
        lift.setTargetPosition(lift.getCurrentPosition() + ((lift.getCurrentPosition() > PowerplayLift.TOLERANCE) ? 6 : 2));
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
        passthrough.claw.setActivated(false);
        setTargetLiftPos(height);
    }

    /**
     * Holds {@link #coneArmR} and {@link #coneArmL} positions
     *
     * @param angleR The angle to turn {@link #coneArmR} to
     * @param angleL The angle to turn {@link #coneArmL} to
     */
    public void run(double angleR, double angleL) {
        passthrough.run();
        if (!clawHasLifted && liftClawTimer.seconds() >= TIME_CLAW) liftClaw();
        coneArmL.turnToAngle(angleL);
        coneArmR.turnToAngle(angleR);
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        lift.printTelemetry(telemetry);
        telemetry.addLine();
        passthrough.printTelemetry(telemetry);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        lift.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        passthrough.printNumericalTelemetry(telemetry);
    }
}
