package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Contains a claw mounted to a wrist on a motion-profiled flip-arm
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class ProfiledClawArm {

    public final SimplePivot claw, wrist;
    private final SimpleServo[] servos;

    private final ElapsedTime profileTimer = new ElapsedTime();

    private MotionProfile profile;

    protected double
            currentAngle, ANGLE_FRONT, ANGLE_BACK,
            ANGLE_PIVOT_POS, TOLERANCE_PIVOT_POS,
            PROFILE_MAX_VELO = 1, PROFILE_MAX_ACCEL = 1, PROFILE_MAX_JERK;

    protected double currentVelocity = 0.0;

    protected boolean inBack = false;
    protected boolean triggered = false;

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update angles and constants
     */
    public ProfiledClawArm(
            SimplePivot claw,
            SimplePivot wrist,
            SimpleServo[] servos
    ) {
        this.claw = claw;
        this.wrist = wrist;
        this.servos = servos;

        profileTimer.reset();

        updateProfile();
    }

    public void updateConstants(
            double ANGLE_FRONT, double ANGLE_BACK,
            double ANGLE_PIVOT_POS, double TOLERANCE_PIVOT_POS,
            double PROFILE_MAX_VELO, double PROFILE_MAX_ACCEL, double PROFILE_MAX_JERK
    ) {
        this.ANGLE_FRONT = ANGLE_FRONT;
        this.ANGLE_BACK = ANGLE_BACK;
        this.ANGLE_PIVOT_POS = ANGLE_PIVOT_POS;
        this.TOLERANCE_PIVOT_POS = TOLERANCE_PIVOT_POS;
        this.PROFILE_MAX_VELO = PROFILE_MAX_VELO;
        this.PROFILE_MAX_ACCEL = PROFILE_MAX_ACCEL;
        this.PROFILE_MAX_JERK = PROFILE_MAX_JERK;
    }

    /**
     * Runs {@link #toggle} and toggles pivot when at the halfway position
     */
    public void trigger() {
        triggered = true;
        toggle();
    }

    /**
     * Toggles position of {@link #servos}
     */
    public void toggle() {
        inBack = !inBack;
        updateProfile();
    }

    /**
     * Updates {@link #profile} with a new target position, including diagonal drop and floor grab tilt presets
     */
    protected void updateProfile() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentAngle, currentVelocity),
                new MotionState(inBack ? ANGLE_BACK : ANGLE_FRONT, 0.0),
                PROFILE_MAX_VELO, PROFILE_MAX_ACCEL, PROFILE_MAX_JERK
        );

        profileTimer.reset();
    }

    /**
     * Hold {@link #servos} positions
     */
    public void run() {
        MotionState state = profile.get(profileTimer.seconds());
        currentAngle = state.getX();
        currentVelocity = state.getV();
        for (SimpleServo servo : servos) {
            servo.turnToAngle(currentAngle);
        }
        if (triggered && Math.abs(ANGLE_PIVOT_POS - currentAngle) <= TOLERANCE_PIVOT_POS) {
            wrist.setActivated(inBack);
            triggered = false;
        }
        wrist.run();
        claw.run();
    }

    public void reset() {
        if (inBack) {
            trigger();
        }
        wrist.setActivated(false);
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
        telemetry.addData("Wrist is oriented to", wrist.getActivated() ? "back" : "front");
        telemetry.addLine();
        telemetry.addData("Arm is in the", inBack ? "back" : "front");
    }
}
