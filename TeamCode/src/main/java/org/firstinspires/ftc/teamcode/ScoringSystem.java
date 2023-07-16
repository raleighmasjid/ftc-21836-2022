package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Passthrough;

/**
 * Merger class, linking a {@link Passthrough} and {@link Lift} by automated methods
 *
 * @author Arshad Anas
 * @since 2022/12/24
 */
@Config
public class ScoringSystem {

    private final SimpleServo coneArmR;
    private final SimpleServo coneArmL;

    private final ElapsedTime liftClawTimer = new ElapsedTime();

    public final Lift lift;

    public final Passthrough passthrough;

    private boolean clawHasLifted = true, floorPickupActivated = false;

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
    public ScoringSystem(HardwareMap hw) {

        lift = new Lift(hw);
        passthrough = new Passthrough(hw);

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

    public void setTargetLiftPos(Lift.Position height) {
        passthrough.setTilt(height == Lift.Position.LOW || height == Lift.Position.MED || height == Lift.Position.TALL);
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
        if (lift.getCurrentPosition() <= (lift.getConesHeight(5) + Lift.TOLERANCE)) {
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
        lift.setTargetPosition(lift.getCurrentPosition() + ((lift.getCurrentPosition() > Lift.TOLERANCE) ? 6 : 2));
        clawHasLifted = true;
    }

    /**
     * Opens claw and runs lift to floor position
     */
    public void dropCone() {
        dropCone(Lift.Position.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     *
     * @param height Named position to run lift to
     */
    public void dropCone(Lift.Position height) {
        passthrough.claw.setActivated(false);
        setTargetLiftPos(height);
    }

    public void toggleFloorPickup() {
        passthrough.toggleTilt();
        floorPickupActivated = !floorPickupActivated;
    }

    /**
     * Holds {@link #coneArmR} and {@link #coneArmL} positions
     *
     * @param angleR The angle to turn {@link #coneArmR} to
     * @param angleL The angle to turn {@link #coneArmL} to
     */
    public void run(double angleR, double angleL) {
        if (floorPickupActivated && passthrough.doneMoving()) {
            passthrough.wrist.toggle();
            passthrough.toggleTilt();
            floorPickupActivated = false;
        }

        if (!clawHasLifted && liftClawTimer.seconds() >= TIME_CLAW) liftClaw();

        coneArmR.turnToAngle(angleR);
        coneArmL.turnToAngle(angleL);
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
