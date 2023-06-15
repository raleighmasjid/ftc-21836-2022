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

    private final SimpleServo pivotServo, servoR, servoL;

    private final ElapsedTime profileTimer = new ElapsedTime();

    private MotionProfile profile;

    private double
            currentAngle, frontAngle, backAngle,
            frontPivotAngle, backPivotAngle,
            pivotPos, pivotPosTolerance,
            tiltOffset, miniTiltOffset,
            maxProfileVelo, maxProfileAccel, maxProfileJerk;

    private double currentVelocity = 0.0;

    private boolean pivotIsFront = true;
    private boolean inFront = true;
    private boolean triggered = false;
    private boolean tilted = false;

    /**
     * Initialize fields, given servo range min = 0
     */
    public ProfiledClawArm(
            double frontAngle, double backAngle,
            double frontPivotAngle, double backPivotAngle,
            double pivotPos, double pivotPosTolerance,
            double tiltOffset, double miniTiltOffset,
            double maxProfileVelo, double maxProfileAccel, double maxProfileJerk,
            SimpleClaw claw,
            SimpleServo pivotServo,
            SimpleServo servoR,
            SimpleServo servoL
    ) {
        updateValues(
                frontAngle, backAngle,
                frontPivotAngle, backPivotAngle,
                pivotPos, pivotPosTolerance,
                tiltOffset, miniTiltOffset,
                maxProfileVelo, maxProfileAccel, maxProfileJerk
        );

        this.claw = claw;
        this.pivotServo = pivotServo;
        this.servoR = servoR;
        this.servoL = servoL;

        profileTimer.reset();

        updateProfile();
    }

    public void updateValues(
            double frontAngle, double backAngle,
            double frontPivotAngle, double backPivotAngle,
            double pivotPos, double pivotPosTolerance,
            double tiltOffset, double miniTiltOffset,
            double maxVelo, double maxAccel, double maxJerk
    ) {
        this.currentAngle = frontAngle;
        this.frontAngle = frontAngle;
        this.backAngle = backAngle;
        this.frontPivotAngle = frontPivotAngle;
        this.backPivotAngle = backPivotAngle;
        this.tiltOffset = tiltOffset;
        this.miniTiltOffset = miniTiltOffset;
        this.pivotPos = pivotPos;
        this.pivotPosTolerance = pivotPosTolerance;
        this.maxProfileVelo = maxVelo;
        this.maxProfileAccel = maxAccel;
        this.maxProfileJerk = maxJerk;
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
        pivotServo.turnToAngle(pivotServo.getAngleRange() - (pivotIsFront ? frontPivotAngle : backPivotAngle));
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
    private void updateProfile() {
        double tiltOffset =
                tilted ?
                        this.tiltOffset :
                        (!triggered) && (inFront != pivotIsFront) ? miniTiltOffset : 0.0;

        double targetAngle =
                inFront ?
                        frontAngle + tiltOffset :
                        backAngle - tiltOffset;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentAngle, currentVelocity),
                new MotionState(targetAngle, 0.0),
                maxProfileVelo, maxProfileAccel, maxProfileJerk
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
        servoL.turnToAngle(servoL.getAngleRange() - currentAngle);
        if (triggered && Math.abs(pivotPos - currentAngle) <= pivotPosTolerance) {
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
