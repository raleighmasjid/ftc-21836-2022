package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.MotionProfiler;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ProfileConstraints;
import org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot;

@Config
public class Passthrough {

    private final SimpleServo[] servos;

    private final MotionProfiler profiler = new MotionProfiler();

    public static double
            ANGLE_CLAW_CLOSED = 0.0,
            ANGLE_CLAW_OPEN = 62.0,
            ANGLE_PASS_FRONT = 8.0,
            ANGLE_PASS_TILT_OFFSET = 45.0,
            ANGLE_PASS_BACK = 310.0,
            ANGLE_PASS_MINI_TILT_OFFSET = 17.0,
            ANGLE_WRIST_FRONT = 17.0,
            ANGLE_WRIST_BACK = 216.0,
            ANGLE_WRIST_PIVOT_POS = (ANGLE_PASS_BACK - ANGLE_PASS_FRONT) * 0.5,
            WRIST_PIVOT_POS_TOLERANCE = 30.0;

    public static ProfileConstraints constraints = new ProfileConstraints(
            600.0,
            1000.0,
            4000.0
    );

    private double currentAngle = ANGLE_PASS_FRONT;
    private double angleOffset = 0.0;

    private boolean tilted = false;
    private boolean triggered = false;
    private boolean inBack = false;

    public final SimpleServoPivot claw, wrist;

    public static SimpleServo axon(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    public static SimpleServo reverseServo(SimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public Passthrough(HardwareMap hw) {
        servos = new SimpleServo[]{axon(hw, "passthrough 1"), reverseServo(axon(hw, "passthrough 2"))};
        claw = new SimpleServoPivot(new SimpleServo[]{axon(hw, "claw right")}, ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        wrist = new SimpleServoPivot(new SimpleServo[]{reverseServo(axon(hw, "claw pivot"))}, ANGLE_WRIST_FRONT, ANGLE_WRIST_BACK);
        updateProfile();
    }

    /**
     * Toggles the value of {@link #tilted}
     */
    public void toggleTilt() {
        setTilt(!tilted);
    }

    /**
     * Sets the value of {@link #tilted} and runs {@link #updateProfile}
     */
    public void setTilt(boolean tilted) {
        this.tilted = tilted;
        updateProfile();
    }

    /**
     * Runs {@link #toggle} and toggles pivot when at the halfway position
     */
    public void trigger() {
        triggered = true;
        toggle();
    }

    /**
     * Toggles the state of the {@link #servos}
     */
    public void toggle() {
        setPosition(!inBack);
    }

    /**
     * Set state of the {@link #servos}
     *
     * @param inBack False for front, true for back
     */
    public void setPosition(boolean inBack) {
        this.inBack = inBack;
        updateProfile();
    }

    private void updateProfile() {
        double tiltOffset =
                tilted ?
                        ANGLE_PASS_TILT_OFFSET :
                        (!triggered) && (inBack != wrist.getActivated()) ? ANGLE_PASS_MINI_TILT_OFFSET : 0.0;

        profiler.updateConstraints(constraints);
        profiler.generateProfile(
                new State(currentAngle, profiler.getV()),
                new State(inBack ?
                        ANGLE_PASS_BACK - tiltOffset :
                        ANGLE_PASS_FRONT + tiltOffset
                )
        );
        angleOffset = 0.0;
    }

    public void run(double manualInput) {
        profiler.update();
        angleOffset += manualInput;
        currentAngle = Math.min(Math.max(profiler.getX() + angleOffset, ANGLE_PASS_FRONT), ANGLE_PASS_BACK);
        for (SimpleServo servo : servos) servo.turnToAngle(currentAngle);

        if (triggered && Math.abs(ANGLE_WRIST_PIVOT_POS - currentAngle) <= WRIST_PIVOT_POS_TOLERANCE) {
            wrist.setActivated(inBack);
            triggered = false;
        }

        claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        claw.run();
        wrist.updateAngles(ANGLE_WRIST_FRONT, ANGLE_WRIST_BACK);
        wrist.run();
    }

    public void run() {
        this.run(0.0);
    }

    public boolean doneMoving() {
        return profiler.isDone();
    }

    public void reset() {
        setPosition(false);
        setTilt(false);
        wrist.setActivated(false);
        claw.setActivated(false);
    }

    /**
     * Print tuning telemetry from {@link #profiler}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Passthrough angle", currentAngle);
        telemetry.addData("Passthrough velocity (ticks/s)", profiler.getV());
    }

    /**
     * Print claw, pivot, and passthrough statuses
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Claw is", claw.getActivated() ? "closed" : "open");
        telemetry.addLine();
        telemetry.addData("Wrist is oriented to", wrist.getActivated() ? "back" : "front");
        telemetry.addLine();
        telemetry.addData("Arm is in the", inBack ? "back" : "front");
    }
}
