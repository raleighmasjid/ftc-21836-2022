package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot;

@Config
public class PowerplayPassthrough {

    private final SimpleServo[] servos;

    private final ElapsedTime profileTimer = new ElapsedTime();

    private MotionProfile profile;

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
            WRIST_PIVOT_POS_TOLERANCE = 30.0,
            PROFILE_MAX_VELO = 600.0,
            PROFILE_MAX_ACCEL = 3000.0,
            PROFILE_MAX_JERK = 7000.0;

    private double currentVelocity, currentAngle = ANGLE_PASS_FRONT;

    private boolean tilted = false;
    private boolean triggered = false;
    private boolean inBack = false;

    public final SimpleServoPivot claw, pivot;

    public static SimpleServo axon(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    public static SimpleServo reverseServo(SimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public PowerplayPassthrough(HardwareMap hw) {
        servos = new SimpleServo[]{axon(hw, "passthrough 1"), reverseServo(axon(hw, "passthrough 2"))};
        claw = new SimpleServoPivot(new SimpleServo[]{axon(hw, "claw right")}, ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        pivot = new SimpleServoPivot(new SimpleServo[]{reverseServo(axon(hw, "claw pivot"))}, ANGLE_WRIST_FRONT, ANGLE_WRIST_BACK);

        profileTimer.reset();
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

    private void updateProfile() {
        double tiltOffset =
                tilted ?
                        ANGLE_PASS_TILT_OFFSET :
                        (!triggered) && (inBack != pivot.getActivated()) ? ANGLE_PASS_MINI_TILT_OFFSET : 0.0;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentAngle, currentVelocity),
                new MotionState(inBack ? ANGLE_PASS_BACK - tiltOffset : ANGLE_PASS_FRONT + tiltOffset, 0.0),
                PROFILE_MAX_VELO, PROFILE_MAX_ACCEL, PROFILE_MAX_JERK
        );

        profileTimer.reset();
    }

    private void updateConstants() {
        claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        pivot.updateAngles(ANGLE_WRIST_FRONT, ANGLE_WRIST_BACK);
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

    /**
     * Get state of the {@link #servos} <p>
     * False for front, true for back
     */
    public boolean getPosition() {
        return inBack;
    }

    public void run() {
        updateConstants();
        MotionState state = profile.get(profileTimer.seconds());
        currentAngle = state.getX();
        currentVelocity = state.getV();
        for (SimpleServo servo : servos) {
            servo.turnToAngle(currentAngle);
        }
        if (triggered && Math.abs(ANGLE_WRIST_PIVOT_POS - currentAngle) <= WRIST_PIVOT_POS_TOLERANCE) {
            pivot.setActivated(inBack);
            triggered = false;
        }
        pivot.run();
        claw.run();
    }

    public void reset() {
        setPosition(false);
        setTilt(false);
        pivot.setActivated(false);
        claw.setActivated(false);
    }

    /**
     * Print tuning telemetry from {@link #profile}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Passthrough angle", currentAngle);
        telemetry.addData("Passthrough velocity (ticks/s)", currentVelocity);
    }

    /**
     * Print claw, pivot, and passthrough statuses
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Claw is", claw.getActivated() ? "closed" : "open");
        telemetry.addLine();
        telemetry.addData("Pivot is oriented to", pivot.getActivated() ? "back" : "front");
        telemetry.addLine();
        telemetry.addData("Arm is in the", inBack ? "back" : "front");
    }
}
