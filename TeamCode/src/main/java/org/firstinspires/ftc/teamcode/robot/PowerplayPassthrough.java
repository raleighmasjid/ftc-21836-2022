package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Contains a multi-function claw and motion-profiled passthrough
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class PowerplayPassthrough {

    private SimpleServo clawServo, pivotServo, servoR, servoL;

    private ElapsedTime profileTimer = new ElapsedTime();

    private MotionProfile profile;

    private double currentAngle = RobotConfig.ANGLE_PASS_FRONT;
    private double currentVelocity = 0.0;

    private boolean pivotIsFront = true;
    private boolean inFront = true;
    private boolean triggered = false;
    private boolean clawIsOpen = true;
    private boolean tilted = false;

    public boolean getClawIsOpen() {
        return clawIsOpen;
    }

    private SimpleServo axonMINI(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    /**
     * Initialize fields
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayPassthrough(HardwareMap hw) {

        clawServo = axonMINI(hw, "claw right");
        pivotServo = axonMINI(hw, "claw pivot");
        servoR = axonMINI(hw, "passthrough 1");
        servoL = axonMINI(hw, "passthrough 2");

        profileTimer.reset();

        updateProfile();
    }

    /**
     * Toggles the value of {@link #clawIsOpen}
     */
    public void toggleClaw() {
        clawIsOpen = !clawIsOpen;
    }

    /**
     * Set state of the claw
     *
     * @param open True if open; false if closed
     */
    public void setClawOpen(boolean open) {
        clawIsOpen = open;
    }

    /**
     * Holds {@link #clawServo} position
     */
    public void runClaw() {
        clawServo.turnToAngle(clawIsOpen ? RobotConfig.ANGLE_CLAW_OPEN : RobotConfig.ANGLE_CLAW_CLOSED);
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
        pivotServo.turnToAngle(355.0 - (pivotIsFront ? RobotConfig.ANGLE_PIVOT_FRONT : RobotConfig.ANGLE_PIVOT_BACK));
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
                        RobotConfig.ANGLE_PASS_TILT_OFFSET :
                        (!triggered) && (inFront != pivotIsFront) ? RobotConfig.ANGLE_PASS_MINI_TILT_OFFSET : 0.0;

        double targetAngle =
                inFront ?
                        RobotConfig.ANGLE_PASS_FRONT + tiltOffset :
                        RobotConfig.ANGLE_PASS_BACK - tiltOffset;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentAngle, currentVelocity),
                new MotionState(targetAngle, 0.0),
                RobotConfig.PASS_MAX_VELO,
                RobotConfig.PASS_MAX_ACCEL,
                RobotConfig.PASS_MAX_JERK
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
        servoL.turnToAngle(355.0 - currentAngle);
        if (triggered && Math.abs(RobotConfig.ANGLE_PIVOT_POS - currentAngle) <= RobotConfig.PASS_PIVOT_POS_TOLERANCE) {
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
     * Print lift, claw, pivot, and passthrough statuses
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Claw is", clawIsOpen ? "open" : "closed");
        telemetry.addLine();
        telemetry.addData("Pivot is oriented to", pivotIsFront ? "front" : "back");
        telemetry.addLine();
        telemetry.addData("Passthrough is", (tilted ? "tilted " : "") + "at the " + (inFront ? "front" : "back"));
    }
}
