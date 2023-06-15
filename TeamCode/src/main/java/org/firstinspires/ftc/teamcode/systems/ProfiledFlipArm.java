package org.firstinspires.ftc.teamcode.systems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Contains a claw mounted to a motion-profiled passthrough
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class ProfiledFlipArm {

    public Claw claw;

    private SimpleServo pivotServo;
    private SimpleServo[] servos;

    private ElapsedTime profileTimer = new ElapsedTime();

    private MotionProfile profile;

    private double
            currentAngle, backPivotAngle, frontPivotAngle, tiltOffset, miniTiltOffset, frontAngle,
            backAngle, maxVelo, maxAccel, maxJerk, pivotPos, pivotPosTolerance;
    private double currentVelocity = 0.0;

    private boolean pivotIsFront = true;
    private boolean inFront = true;
    private boolean triggered = false;
    private boolean tilted = false;

    /**
     * Initialize fields
     */
    public ProfiledFlipArm(
            double frontAngle,
            double backAngle,
            double frontPivotAngle,
            double backPivotAngle,
            double pivotPos,
            double pivotPosTolerance,
            double tiltOffset,
            double miniTiltOffset,
            double maxVelo,
            double maxAccel,
            double maxJerk,
            Claw claw,
            SimpleServo pivotServo,
            SimpleServo... servos
    ) {
        updateValues(frontAngle, backAngle, frontPivotAngle, backPivotAngle, pivotPos, pivotPosTolerance, tiltOffset, miniTiltOffset, maxVelo, maxAccel, maxJerk);

        this.claw = claw;
        this.pivotServo = pivotServo;
        this.servos = servos;

        profileTimer.reset();

        updateProfile();
    }

    public void updateValues(
            double frontAngle,
            double backAngle,
            double frontPivotAngle,
            double backPivotAngle,
            double pivotPos,
            double pivotPosTolerance,
            double tiltOffset,
            double miniTiltOffset,
            double maxVelo,
            double maxAccel,
            double maxJerk
    ) {
        this.currentAngle = frontPivotAngle;
        this.frontPivotAngle = frontPivotAngle;
        this.backPivotAngle = backPivotAngle;
        this.tiltOffset = tiltOffset;
        this.miniTiltOffset = miniTiltOffset;
        this.frontAngle = frontAngle;
        this.backAngle = backAngle;
        this.pivotPos = pivotPos;
        this.pivotPosTolerance = pivotPosTolerance;
        this.maxVelo = maxVelo;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;
    }

    /**
     * Toggles the value of {@link #pivotIsFront}
     */
    public void togglePivot() {
        setPivotIsFront(!pivotIsFront);
    }

    /**
     * Sets the value of {@link #pivotIsFront}
     */
    public void setPivotIsFront(boolean isFront) {
        pivotIsFront = isFront;
        updateProfile();
    }

    /**
     * Holds pivot servo position
     */
    public void runPivot() {
        pivotServo.turnToAngle(355.0 - (pivotIsFront ? frontPivotAngle : backPivotAngle));
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
     * Toggles position of {@link #servos}
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
                maxVelo, maxAccel, maxJerk
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
        servos[0].turnToAngle(currentAngle);
        servos[1].turnToAngle(servos[1].getAngleRange() - currentAngle);
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
