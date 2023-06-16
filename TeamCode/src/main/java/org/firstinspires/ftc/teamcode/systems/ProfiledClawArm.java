package org.firstinspires.ftc.teamcode.systems;


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

    public final SimpleClaw claw;

    protected final SimpleServo pivotServo, servoR, servoL;

    protected final ElapsedTime profileTimer = new ElapsedTime();

    protected MotionProfile profile;

    protected double
            currentAngle, ANGLE_FRONT, ANGLE_BACK,
            ANGLE_PIVOT_FRONT, ANGLE_PIVOT_BACK,
            ANGLE_PIVOT_POS, TOLERANCE_PIVOT_POS,
            ANGLE_TILT_OFFSET, ANGLE_MINI_TILT_OFFSET,
            PROFILE_MAX_VELO, PROFILE_MAX_ACCEL, PROFILE_MAX_JERK;

    protected double currentVelocity = 0.0;

    protected boolean pivotIsFront = true;
    protected boolean inFront = true;
    protected boolean triggered = false;
    protected boolean tilted = false;

    public ProfiledClawArm(SimpleClaw claw, SimpleServo pivotServo, SimpleServo servo) {
        this(claw, pivotServo, servo, null);
    }

    /**
     * Initialize fields <p>
     * Use {@link #updateAngles} and {@link #updateConstants} to update angles and constants, respectively
     */
    public ProfiledClawArm(
            SimpleClaw claw,
            SimpleServo pivotServo,
            SimpleServo servoR,
            SimpleServo servoL
    ) {
        this.claw = claw;
        this.pivotServo = pivotServo;
        this.servoR = servoR;
        this.servoL = servoL;

        profileTimer.reset();

        updateProfile();
    }

    public void updateAngles(
            double ANGLE_FRONT, double ANGLE_BACK,
            double ANGLE_PIVOT_FRONT, double ANGLE_PIVOT_BACK,
            double ANGLE_PIVOT_POS,
            double ANGLE_TILT_OFFSET, double ANGLE_MINI_TILT_OFFSET
    ) {
        this.currentAngle = ANGLE_FRONT;
        this.ANGLE_FRONT = ANGLE_FRONT;
        this.ANGLE_BACK = ANGLE_BACK;
        this.ANGLE_PIVOT_FRONT = ANGLE_PIVOT_FRONT;
        this.ANGLE_PIVOT_BACK = ANGLE_PIVOT_BACK;
        this.ANGLE_TILT_OFFSET = ANGLE_TILT_OFFSET;
        this.ANGLE_MINI_TILT_OFFSET = ANGLE_MINI_TILT_OFFSET;
        this.ANGLE_PIVOT_POS = ANGLE_PIVOT_POS;
    }

    public void updateConstants(double PIVOT_POS_TOLERANCE, double PROFILE_MAX_VELO, double PROFILE_MAX_ACCEL, double PROFILE_MAX_JERK) {
        this.TOLERANCE_PIVOT_POS = PIVOT_POS_TOLERANCE;
        this.PROFILE_MAX_VELO = PROFILE_MAX_VELO;
        this.PROFILE_MAX_ACCEL = PROFILE_MAX_ACCEL;
        this.PROFILE_MAX_JERK = PROFILE_MAX_JERK;
    }

    /**
     * Toggles the value of {@link #pivotIsFront}
     */
    public void togglePivot() {
        setPivotState(!pivotIsFront);
    }

    /**
     * Set state of {@link #pivotServo}
     *
     * @param isFront True if oriented to front; false if oriented to back
     */
    public void setPivotState(boolean isFront) {
        pivotIsFront = isFront;
        updateProfile();
    }

    /**
     * Holds pivot servo position
     */
    public void runPivot() {
        pivotServo.turnToAngle(pivotServo.getAngleRange() - (pivotIsFront ? ANGLE_PIVOT_FRONT : ANGLE_PIVOT_BACK));
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
     * Toggles position of {@link #servoR} and {@link #servoL}
     */
    public void toggle() {
        inFront = !inFront;
        updateProfile();
    }

    /**
     * Updates {@link #profile} with a new target position, including diagonal drop and floor grab tilt presets
     */
    protected void updateProfile() {
        double tiltOffset =
                tilted ?
                        this.ANGLE_TILT_OFFSET :
                        (!triggered) && (inFront != pivotIsFront) ? ANGLE_MINI_TILT_OFFSET : 0.0;

        double targetAngle =
                inFront ?
                        ANGLE_FRONT + tiltOffset :
                        ANGLE_BACK - tiltOffset;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentAngle, currentVelocity),
                new MotionState(targetAngle, 0.0),
                PROFILE_MAX_VELO, PROFILE_MAX_ACCEL, PROFILE_MAX_JERK
        );

        profileTimer.reset();
    }

    /**
     * Hold {@link #servoR} and {@link #servoL} positions
     */
    public void run() {
        MotionState state = profile.get(profileTimer.seconds());
        currentAngle = state.getX();
        currentVelocity = state.getV();
        servoR.turnToAngle(currentAngle);
        if (servoL != null) {
            servoL.turnToAngle(currentAngle);
        }
        if (triggered && Math.abs(ANGLE_PIVOT_POS - currentAngle) <= TOLERANCE_PIVOT_POS) {
            pivotIsFront = inFront;
            triggered = false;
        }
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
        claw.printTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("Pivot is oriented to", pivotIsFront ? "front" : "back");
        telemetry.addLine();
        telemetry.addData("Passthrough is", (tilted ? "tilted " : "") + "at the " + (inFront ? "front" : "back"));
    }
}
