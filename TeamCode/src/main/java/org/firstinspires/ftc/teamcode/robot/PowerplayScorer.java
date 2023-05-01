package org.firstinspires.ftc.teamcode.robot;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.LowPassFilter;
import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.jetbrains.annotations.Contract;

/**
 * Contains 3-motor automated lift, 3-state claw, and state-machine-controlled passthrough functions
 *
 * @author Arshad Anas
 * @since 2022/12/24
 */
public class PowerplayScorer {
    /**
     * Motor powering the dual lift system
     */
    private final MotorEx lift_motor1, lift_motor2, lift_motor3;
    private final SimpleServo clawServo, pivotServo, passThruServoR, passThruServoL, coneArmServoR, coneArmServoL;
    /**
     * PID + feedforward controller for lift
     */
    private final PIDFController liftController;
    /**
     * Lift motion profile to track along
     */
    private MotionProfile liftProfile;
    /**
     * Timer for tracking along lift motion profile
     */
    private final ElapsedTime liftProfileTimer;
    /**
     * Lift state grabbed from motion profile
     */
    private MotionState profileLiftState;
    /**
     * Current lift state
     */
    private MotionState currentLiftState;
    /**
     * Named end target position
     */
    private String targetLiftPosName;
    /**
     * Timer for differentiating velocity, acceleration, and jerk
     */
    private final ElapsedTime liftDerivTimer;
    /**
     * Timer to track claw closing time before lifting
     */
    private static ElapsedTime liftClawTimer;
    /**
     * True by default
     * False only if grabCone has been called, and liftClaw has not been called
     */
    private boolean clawHasLifted;
    /**
     * Desired pivot state
     * True if pivot should be oriented for its front position
     * False if oriented for back position
     */
    private boolean pivotIsFront;
    /**
     * False by default
     * True only if passthrough sequence has not yet completed
     */
    private boolean passThruIsMoving;
    /**
     * False by default
     * True only if passthrough sequence was triggered while it was still moving
     */
    private boolean passThruSwitched;
    /**
     * Supposed state of the passthrough
     * True if front position has been reached
     * False if back position has been reached
     */
    private boolean passThruInFront;
    /**
     * Timer to track sequential passthrough events
     */
    private static ElapsedTime passThruTimer;
    /**
     * Cached state of cone-flipping arms
     */
    private boolean coneArmsAreDown;

    /**
     * State of the passthrough sequence
     */
    private enum passThruState {
        START, FRONT, FRONT_PIVOT, PIVOTING, BACK_PIVOT, BACK
    }

    /**
     * Named position of main passthrough servos
     */
    private enum passThruPos {
        FRONT, PIVOT_POS, BACK
    }

    /**
     * Named lift position
     */
    public enum liftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    private final LowPassFilter jerkFilter, accelFilter, veloFilter;
    private double currentTimestamp;
    private boolean clawIsOpen;
    private boolean clawIsTilted;
    private passThruState currentPassThruState;
    private passThruPos currentPassThruPos;

    @NonNull
    @Contract("_, _ -> new")
    private SimpleServo axonMINI(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    @NonNull
    @Contract("_, _ -> new")
    private SimpleServo goBILDAServo(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 280);
    }

    @NonNull
    @Contract("_, _ -> new")
    private MotorEx liftMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 145.1, 1150);
    }

    /**
     * Initialize internal objects and variables
     *
     * @param hw Passed-in hardware map from the op mode
     */
    public PowerplayScorer(HardwareMap hw) {

        clawServo = axonMINI(hw, "claw right");
        pivotServo = axonMINI(hw, "claw pivot");
        passThruServoR = axonMINI(hw, "passthrough 1");
        passThruServoL = axonMINI(hw, "passthrough 2");
        coneArmServoR = goBILDAServo(hw, "arm right");
        coneArmServoL = goBILDAServo(hw, "arm left");

        lift_motor1 = liftMotor(hw, "lift motor 1");
        lift_motor2 = liftMotor(hw, "lift motor 2");
        lift_motor3 = liftMotor(hw, "lift motor 3");

        lift_motor1.setInverted(true);
        lift_motor2.setInverted(false);
        lift_motor3.setInverted(true);

        lift_motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        lift_motor1.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor2.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor3.setRunMode(Motor.RunMode.VelocityControl);

        liftController = new PIDFController(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_INTEGRATION_MAX_VELO,
                RobotConfig.LIFT_PID_FILTER_GAIN,
                RobotConfig.LIFT_PID_ESTIMATE_COUNT,
                RobotConfig.LIFT_kV,
                RobotConfig.LIFT_kA,
                RobotConfig.LIFT_kS
        );
        liftController.setPositionTolerance(RobotConfig.LIFT_POS_TOLERANCE);
        liftController.setOutputBounds(-1.0, 1.0);

        veloFilter = new LowPassFilter();
        accelFilter = new LowPassFilter();
        jerkFilter = new LowPassFilter();

        veloFilter.setGains(RobotConfig.LIFT_VELO_FILTER_GAIN, RobotConfig.LIFT_VELO_ESTIMATE_COUNT);
        accelFilter.setGains(RobotConfig.LIFT_ACCEL_FILTER_GAIN, RobotConfig.LIFT_ACCEL_ESTIMATE_COUNT);
        jerkFilter.setGains(RobotConfig.LIFT_JERK_FILTER_GAIN, RobotConfig.LIFT_JERK_ESTIMATE_COUNT);

        clawHasLifted = true;
        pivotIsFront = true;
        passThruInFront = true;
        passThruIsMoving = false;
        clawIsOpen = true;
        clawIsTilted = false;
        passThruSwitched = false;
        coneArmsAreDown = false;
        currentPassThruPos = passThruPos.FRONT;
        currentPassThruState = passThruState.FRONT;

        passThruTimer = new ElapsedTime();
        passThruTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();
        liftProfileTimer = new ElapsedTime();
        liftProfileTimer.reset();
        liftDerivTimer = new ElapsedTime();
        liftDerivTimer.reset();

        resetLift();
    }

    /**
     * @param angle Angle to turn main passthrough servos to
     */
    private void setPassThruAngle(double angle) {
        passThruServoR.turnToAngle(angle);
        passThruServoL.turnToAngle(355.0 - angle);
    }

    /**
     * Hold main passthrough servo positions
     */
    public void runPassThruServos() {
        switch (currentPassThruPos) {
            case FRONT:
                setPassThruAngle(RobotConfig.ANGLE_PASS_FRONT + (clawIsTilted ? RobotConfig.ANGLE_PASS_TILT : 0.0));
                break;
            case PIVOT_POS:
                setPassThruAngle(RobotConfig.ANGLE_PASS_PIVOT);
                break;
            case BACK:
                setPassThruAngle(RobotConfig.ANGLE_PASS_BACK - (clawIsTilted ? RobotConfig.ANGLE_PASS_TILT : 0.0));
                break;
            default:
                currentPassThruPos = passThruPos.FRONT;
                break;
        }
    }

    /**
     * Run automated passthrough sequence
     * Triggered by triggerPassThru()
     */
    public void runPassThruStates() {
        if (passThruInFront) {
            switch (currentPassThruState) {
                default:
                case BACK:
                case FRONT:
                    passThruIsMoving = false;
                    passThruTimer.reset();
                    break;
                case START:
                    currentPassThruState = passThruState.FRONT_PIVOT;
                    break;
                case FRONT_PIVOT:
                    if (passThruSwitched) currentPassThruPos = passThruPos.PIVOT_POS;
                    if (passThruTimer.seconds() >= RobotConfig.TIME_FRONT_PIVOT) {
                        pivotIsFront = false;
                        passThruTimer.reset();
                        currentPassThruState = passThruState.PIVOTING;
                    }
                    break;
                case PIVOTING:
                    if (passThruSwitched) pivotIsFront = false;
                    if (passThruTimer.seconds() >= RobotConfig.TIME_PIVOTING) {
                        passThruTimer.reset();
                        currentPassThruPos = passThruPos.BACK;
                        currentPassThruState = passThruState.BACK_PIVOT;
                    }
                    break;
                case BACK_PIVOT:
                    if (passThruSwitched) currentPassThruPos = passThruPos.BACK;
                    if (passThruTimer.seconds() >= RobotConfig.TIME_BACK_PIVOT) {
                        passThruInFront = false;
                        passThruTimer.reset();
                        currentPassThruState = passThruState.BACK;
                    }
                    break;
            }
        } else {
            switch (currentPassThruState) {
                default:
                case FRONT:
                case BACK:
                    passThruIsMoving = false;
                    passThruTimer.reset();
                    break;
                case START:
                    currentPassThruState = passThruState.BACK_PIVOT;
                    break;
                case BACK_PIVOT:
                    if (passThruSwitched) currentPassThruPos = passThruPos.PIVOT_POS;
                    if (passThruTimer.seconds() >= RobotConfig.TIME_BACK_PIVOT) {
                        pivotIsFront = true;
                        passThruTimer.reset();
                        currentPassThruState = passThruState.PIVOTING;
                    }
                    break;
                case PIVOTING:
                    if (passThruSwitched) pivotIsFront = true;
                    if (passThruTimer.seconds() >= RobotConfig.TIME_PIVOTING) {
                        passThruTimer.reset();
                        currentPassThruPos = passThruPos.FRONT;
                        currentPassThruState = passThruState.FRONT_PIVOT;
                    }
                    break;
                case FRONT_PIVOT:
                    if (passThruSwitched) currentPassThruPos = passThruPos.FRONT;
                    if (passThruTimer.seconds() >= RobotConfig.TIME_FRONT_PIVOT) {
                        passThruInFront = true;
                        passThruTimer.reset();
                        currentPassThruState = passThruState.FRONT;
                    }
                    break;
            }
        }
        passThruSwitched = false;
    }

    /**
     * Set target for lift motion profile
     *
     * @param height Desired named position to run to
     */
    public void setTargetLiftPos(@NonNull liftPos height) {
        clawIsTilted = height == liftPos.LOW || height == liftPos.MED || height == liftPos.TALL;
        double targetLiftPos;
        switch (height) {
            default:
            case FLOOR:
                targetLiftPos = RobotConfig.HEIGHT_FLOOR;
                targetLiftPosName = liftPos.FLOOR.name();
                break;
            case TWO:
                targetLiftPos = RobotConfig.HEIGHT_TWO;
                targetLiftPosName = liftPos.TWO.name();
                break;
            case THREE:
                targetLiftPos = RobotConfig.HEIGHT_THREE;
                targetLiftPosName = liftPos.THREE.name();
                break;
            case FOUR:
                targetLiftPos = RobotConfig.HEIGHT_FOUR;
                targetLiftPosName = liftPos.FOUR.name();
                break;
            case FIVE:
                targetLiftPos = RobotConfig.HEIGHT_FIVE;
                targetLiftPosName = liftPos.FIVE.name();
                break;
            case LOW:
                targetLiftPos = RobotConfig.HEIGHT_LOW;
                targetLiftPosName = liftPos.LOW.name();
                break;
            case MED:
                targetLiftPos = RobotConfig.HEIGHT_MEDIUM;
                targetLiftPosName = liftPos.MED.name();
                break;
            case TALL:
                targetLiftPos = RobotConfig.HEIGHT_TALL;
                targetLiftPosName = liftPos.TALL.name();
                break;
        }
        updateLiftProfile(targetLiftPos);
    }

    /**
     * Set target for lift motion profile
     *
     * @param targetLiftPos Desired position (in inches) to run to
     */
    public void setTargetLiftPos(double targetLiftPos) {
        targetLiftPosName = Double.toString(targetLiftPos);
        updateLiftProfile(targetLiftPos);
    }

    /**
     * Sets the target lift state to the current lift state
     */
    public void setLiftStateToCurrent() {
        setTargetLiftPos(currentLiftState.getX());
    }

    /**
     * Update lift motion profile with a new target position
     */
    private void updateLiftProfile(double targetLiftPos) {
        boolean goingDown = targetLiftPos < currentLiftState.getX();

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                currentLiftState,
                new MotionState(targetLiftPos, 0, 0, 0),
                goingDown ? RobotConfig.LIFT_MAX_DOWN_VELO : RobotConfig.LIFT_MAX_UP_VELO,
                goingDown ? RobotConfig.LIFT_MAX_DOWN_ACCEL : RobotConfig.LIFT_MAX_UP_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );

        liftProfileTimer.reset();
    }

    /**
     * Update lift PIDF controller gains with constants from RobotConfig.java
     */
    private void updateLiftGains() {
        liftController.setGains(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_INTEGRATION_MAX_VELO,
                RobotConfig.LIFT_PID_FILTER_GAIN,
                RobotConfig.LIFT_PID_ESTIMATE_COUNT,
                RobotConfig.LIFT_kV,
                RobotConfig.LIFT_kA,
                RobotConfig.LIFT_kS
        );
        liftController.setPositionTolerance(RobotConfig.LIFT_POS_TOLERANCE);
    }

    /**
     * Reads lift encoder value and converts to position in inches
     * Calculates velocity, acceleration, and jerk
     * Saves readings to currentLiftState
     */
    public void readLiftPos() {
        double
                newTimestamp = liftDerivTimer.seconds(),
                dt = newTimestamp - currentTimestamp;
        boolean dtIsZero = dt == 0.0;
        currentTimestamp = newTimestamp;

        veloFilter.setGains(RobotConfig.LIFT_VELO_FILTER_GAIN, RobotConfig.LIFT_VELO_ESTIMATE_COUNT);
        accelFilter.setGains(RobotConfig.LIFT_ACCEL_FILTER_GAIN, RobotConfig.LIFT_ACCEL_ESTIMATE_COUNT);
        jerkFilter.setGains(RobotConfig.LIFT_JERK_FILTER_GAIN, RobotConfig.LIFT_JERK_ESTIMATE_COUNT);

        double newLiftPos = lift_motor2.encoder.getPosition() * RobotConfig.LIFT_INCHES_PER_TICK;
        double newLiftVelo = dtIsZero ? 0.0 : (veloFilter.getEstimate((newLiftPos - currentLiftState.getX()) / dt));
        double newLiftAccel = dtIsZero ? 0.0 : (accelFilter.getEstimate((newLiftVelo - currentLiftState.getV()) / dt));
        double newLiftJerk = dtIsZero ? 0.0 : (jerkFilter.getEstimate((newLiftAccel - currentLiftState.getA()) / dt));

        currentLiftState = new MotionState(newLiftPos, newLiftVelo, newLiftAccel, newLiftJerk);
        updateLiftGains();
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        jerkFilter.resetPastValues();
        accelFilter.resetPastValues();
        veloFilter.resetPastValues();

        currentTimestamp = 0.0;
        targetLiftPosName = liftPos.FLOOR.name();
        clawIsTilted = false;

        liftDerivTimer.reset();
        liftProfileTimer.reset();
        liftController.reset();
        lift_motor2.resetEncoder();

        currentLiftState = new MotionState(0.0, 0.0, 0.0, 0.0);
        profileLiftState = new MotionState(0.0, 0.0, 0.0, 0.0);

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0.0, 0.0, 0.0, 0.0),
                new MotionState(0.0, 0.0, 0.0, 0.0),
                RobotConfig.LIFT_MAX_UP_VELO,
                RobotConfig.LIFT_MAX_UP_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );
    }

    /**
     * Runs lift PIDF controller to track along motion profile
     */
    public void runLiftToPos() {
        double currentLiftPos = currentLiftState.getX();
        profileLiftState = liftProfile.get(liftProfileTimer.seconds());

        liftController.setTargetPosition(profileLiftState.getX());
        liftController.setTargetVelocity(profileLiftState.getV());
        liftController.setTargetAcceleration(profileLiftState.getA());

        if (liftController.atTargetPosition(currentLiftPos)) liftController.reset();

        runLift(liftController.update(currentLiftPos));
    }

    /**
     * Run lift motors
     *
     * @param veloCommand Pass in a velocity between 0 and 1
     */
    public void runLift(double veloCommand) {
        veloCommand += kG();
        lift_motor1.set(veloCommand);
        lift_motor2.set(veloCommand);
        lift_motor3.set(veloCommand);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     *
     * @return Velocity command for lift
     */
    private double kG() {
        double currentLiftPos = currentLiftState.getX();
        return currentLiftPos >= RobotConfig.HEIGHT_STAGES_FOUR ? RobotConfig.LIFT_kG_FOUR :
                currentLiftPos >= RobotConfig.HEIGHT_STAGES_THREE ? RobotConfig.LIFT_kG_THREE :
                        currentLiftPos >= RobotConfig.HEIGHT_STAGES_TWO ? RobotConfig.LIFT_kG_TWO :
                                currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE ? RobotConfig.LIFT_kG_ONE :
                                        0.0;
    }

    public void toggleClawTilt() {
        clawIsTilted = !clawIsTilted;
    }

    public void toggleClaw() {
        clawIsOpen = !clawIsOpen;
    }

    /**
     * Closes and lift claw if open
     * Opens and lowers claw if already closed
     */
    public void triggerClaw() {
        if (clawIsOpen) grabCone();
        else dropCone();
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
     * Holds claw servo position
     */
    public void runClaw() {
        clawServo.turnToAngle(
                clawIsOpen ?
                        passThruIsMoving ? // open
                                RobotConfig.ANGLE_CLAW_PASS : // moving
                                RobotConfig.ANGLE_CLAW_OPEN : // not moving
                        RobotConfig.ANGLE_CLAW_CLOSED // closed
        );

        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
    }

    /**
     * Closes claw
     * Waits for claw to close
     * Lifts claw
     */
    public void grabCone() {
        setClawOpen(false);
        if (currentLiftState.getX() <= (RobotConfig.HEIGHT_FIVE + RobotConfig.LIFT_POS_TOLERANCE)) {
            clawHasLifted = false;
            liftClawTimer.reset();
        }
    }

    /**
     * Lifts claw either:
     * 6 inches if grabbing off stack
     * 2 inches if grabbing off the floor
     */
    public void liftClaw() {
        double currentLiftPos = currentLiftState.getX();
        setTargetLiftPos(currentLiftPos + ((currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE) ? 6 : 2));
        clawHasLifted = true;
    }

    /**
     * Opens claw and runs lift to floor position
     */
    public void dropCone() {
        dropCone(liftPos.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     *
     * @param height Named position to run lift to
     */
    public void dropCone(liftPos height) {
        setClawOpen(true);
        setTargetLiftPos(height);
    }

    public void togglePivot() {
        pivotIsFront = !pivotIsFront;
    }

    /**
     * Holds pivot servo position
     */
    public void runPivot() {
        pivotServo.turnToAngle(355.0 - (pivotIsFront ? RobotConfig.ANGLE_PIVOT_FRONT : RobotConfig.ANGLE_PIVOT_BACK));
    }

    /**
     * Activates automated passthrough sequence
     */
    public void triggerPassThru() {
        if (passThruIsMoving) {
            passThruInFront = !passThruInFront;
            passThruSwitched = true;
        } else {
            passThruTimer.reset();
            passThruIsMoving = true;
            currentPassThruPos = passThruPos.PIVOT_POS;
            currentPassThruState = passThruState.START;
        }
    }

    /**
     * Toggles position of main passthrough servos
     */
    public void togglePassThru() {
        currentPassThruPos = passThruInFront ? passThruPos.BACK : passThruPos.FRONT;
        currentPassThruState = passThruInFront ? passThruState.BACK : passThruState.FRONT;
        passThruInFront = !passThruInFront;
    }

    /**
     * Holds cone arm servos in position
     *
     * @param down True if arms are to be down; false if arms should be upright
     */
    public void runConeArms(boolean down) {
        coneArmsAreDown = down;
        double angle = down ? RobotConfig.ANGLE_ARM_DOWN : RobotConfig.ANGLE_ARM_UP;
        coneArmServoL.turnToAngle(280.0 - angle);
        coneArmServoR.turnToAngle(angle);
    }

    /**
     * Print relevant telemetry of the system
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(@NonNull MultipleTelemetry telemetry) {
        telemetry.addData("Lift raw encoder reading", lift_motor2.encoder.getPosition());
        telemetry.addData("Named target lift position", targetLiftPosName);
        telemetry.addLine();
        telemetry.addData("Lift current position (in)", currentLiftState.getX());
        telemetry.addData("Lift profile position (in)", profileLiftState.getX());
        telemetry.addData("Lift position error (in)", liftController.getCurrentFilterEstimate());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentLiftState.getV());
        telemetry.addData("Lift profile velocity (in/s)", profileLiftState.getV());
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentLiftState.getA());
        telemetry.addLine();
        telemetry.addData("Lift current jerk (in/s^3)", currentLiftState.getJ());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Cone arms are", coneArmsAreDown ? "down" : "up");
        telemetry.addLine();
        telemetry.addData("Claw is", clawIsOpen ? "open" : "closed");
        telemetry.addLine();
        telemetry.addData("Pivot is oriented to", pivotIsFront ? "front" : "back");
        telemetry.addLine();
        telemetry.addData("Passthrough status", currentPassThruState);
    }
}
