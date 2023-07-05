package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Servo(s) with two set positions <p>
 * Controlled by {@link #toggle}
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class ProfiledPivot {

    private final SimpleServo[] servos;

    private final ElapsedTime profileTimer = new ElapsedTime();

    private MotionProfile profile;

    private double PROFILE_MAX_VELO = 1, PROFILE_MAX_ACCEL = 1, PROFILE_MAX_JERK;

    protected double currentAngle, currentVelocity, ANGLE_A, ANGLE_B;

    protected boolean activated = false;

    /**
     * Initialize fields <p>
     * Use {@link #updateConstants} to update angles and constants
     */
    public ProfiledPivot(SimpleServo[] servos) {
        this.servos = servos;
        profileTimer.reset();
        updateProfile();
    }

    public void updateConstants(
            double ANGLE_FRONT, double ANGLE_BACK,
            double PROFILE_MAX_VELO, double PROFILE_MAX_ACCEL, double PROFILE_MAX_JERK
    ) {
        this.ANGLE_A = ANGLE_FRONT;
        this.ANGLE_B = ANGLE_BACK;
        this.PROFILE_MAX_VELO = PROFILE_MAX_VELO;
        this.PROFILE_MAX_ACCEL = PROFILE_MAX_ACCEL;
        this.PROFILE_MAX_JERK = PROFILE_MAX_JERK;
    }

    /**
     * Updates {@link #profile} with a new target position, including diagonal drop and floor grab tilt presets
     */
    protected void updateProfile() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentAngle, currentVelocity),
                new MotionState(activated ? ANGLE_B : ANGLE_A, 0.0),
                PROFILE_MAX_VELO, PROFILE_MAX_ACCEL, PROFILE_MAX_JERK
        );
        profileTimer.reset();
    }

    /**
     * Toggles the state of the {@link #servos}
     */
    public void toggle() {
        setActivated(!activated);
    }

    /**
     * Set state of the {@link #servos}
     *
     * @param activated False for position A, true for position B
     */
    public void setActivated(boolean activated) {
        this.activated = activated;
        updateProfile();
    }

    /**
     * Get state of the {@link #servos} <p>
     * False if position A (default) <p>
     * True if in position B
     */
    public boolean getActivated() {
        return activated;
    }

    /**
     * Hold {@link #servos} position
     */
    public void run() {
        MotionState state = profile.get(profileTimer.seconds());
        currentAngle = state.getX();
        currentVelocity = state.getV();
        for (SimpleServo servo : servos) {
            servo.turnToAngle(currentAngle);
        }
    }

    public void reset() {
        if (activated) {
            toggle();
        }
    }

    /**
     * Print tuning telemetry from {@link #profile}
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Arm angle", currentAngle);
        telemetry.addData("Arm velocity (ticks/s)", currentVelocity);
    }

    /**
     * Print claw, pivot, and passthrough statuses
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Arm is in the", activated ? "back" : "front");
    }
}
