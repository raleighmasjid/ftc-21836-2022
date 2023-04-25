package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
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

/**
 * Contains 3-motor automated lift, 3-state claw, and state-machine-controlled passthrough functions
 * @author Arshad Anas
 * @since 2022/12/24
 */
public class PowerplayScorer {
    /**
     * Motor geared with two others to power the dual lift system
     */
    private MotorEx
            lift_motor1,
            lift_motor2,
            lift_motor3;
    /**
     * Servo
     */
    private SimpleServo
            clawServo,
            pivotServo,
            passThruServoR,
            passThruServoL,
            coneArmServoR,
            coneArmServoL;
    /**
     * PID + feedforward controller for lift
     */
    private PIDFController liftController;
    /**
     * Lift motion profile to track along
     */
    private MotionProfile liftProfile;
    /**
     * Latest lift state grabbed from
     */
    private MotionState liftProfileState;
    /**
     * Name of the named position to run lift to
     */
    private String targetLiftPosName;
    /**
     * Timer for tracking along lift motion profile
     */
    private ElapsedTime liftProfileTimer;
    /**
     * Timer for differentiating velocity, acceleration, and jerk
     */
    private ElapsedTime liftDerivTimer;
    /**
     * Timer to track sequential passthrough events
     */
    private static ElapsedTime passThruTimer;
    /**
     * Passthrough to track claw closing time before lifting
     */
    private static ElapsedTime liftClawTimer;
    /**
     * Finite impulse response low-pass filter
     * Filters out sensor noise amplified through differentiation
     */
    private LowPassFilter
            jerkFilter,
            accelFilter,
            veloFilter;
    /**
     * Timestamp of the last loop
     * Subtracted from the current timestamp to obtain dt for differentiation
     */
    private double
            lastTimestamp,
            currentLiftJerk,
            currentLiftAccel,
            currentLiftVelo,
            currentLiftPos,
            targetLiftPos;
    /**
     * Count of ticks per revolution for lift motors
     */
    private static final double LIFT_TICKS = 145.1;
    /**
     * Revolutions per minute for lift motors
     */
    private static final double LIFT_RPM = 1150;
    /**
     * Desired claw position
     * True if open
     * False if closed
     */
    private boolean clawIsOpen;
    /**
     * True by default
     * False only if liftClaw has been called, and the claw has not yet lifted
     */
    private boolean clawHasLifted;
    /**
     * Desired tilt state
     * True if tilted
     * False if down in default position
     */
    private boolean clawIsTilted;
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
     * Initialize internal objects and variables
     * @param hw Passed-in hardware map from the op mode
     */
    public void init (HardwareMap hw) {

        clawServo = new SimpleServo(hw,"claw right",0,355);
        pivotServo = new SimpleServo(hw, "claw pivot",0,355);
        passThruServoR = new SimpleServo(hw, "passthrough 1",0,355);
        passThruServoL = new SimpleServo(hw, "passthrough 2",0,355);
        coneArmServoR = new SimpleServo(hw, "arm right",0,280);
        coneArmServoL = new SimpleServo(hw, "arm left",0,280);

        lift_motor1 = new MotorEx(hw, "lift motor 1", LIFT_TICKS, LIFT_RPM);
        lift_motor2 = new MotorEx(hw, "lift motor 2", LIFT_TICKS, LIFT_RPM);
        lift_motor3 = new MotorEx(hw, "lift motor 3", LIFT_TICKS, LIFT_RPM);

        liftController = new PIDFController(
                new PIDCoefficients(
                        RobotConfig.LIFT_kP,
                        RobotConfig.LIFT_kI,
                        RobotConfig.LIFT_kD
                ),
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

        lift_motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        lift_motor1.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor2.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor3.setRunMode(Motor.RunMode.VelocityControl);

        lift_motor1.setInverted(true);
        lift_motor2.setInverted(false);
        lift_motor3.setInverted(true);

        clawHasLifted = true;
        pivotIsFront = true;
        passThruInFront = true;
        passThruIsMoving = false;
        setClawOpen(true);
        clawIsTilted = false;
        passThruSwitched = false;

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
     * State of the passthrough sequence
     */
    private enum passThruState {
        START,
        FRONT,
        FRONT_PIVOT,
        PIVOTING,
        BACK_PIVOT,
        BACK
    }

    /**
     * Named position of main passthrough servos
     */
    private enum passThruPos {
        FRONT_IDLE,
        FRONT,
        PIVOT_POS,
        BACK,
        BACK_IDLE
    }

    /**
     * Current state of passthrough sequence
     */
    private passThruState currentPassThruState = passThruState.FRONT;
    /**
     * Current position of main passthrough servos
     */
    private passThruPos currentPassThruPos = passThruPos.FRONT_IDLE;

    /**
     * @param angle Angle to turn main passthrough servos to
     */
    private void setPassThruAngle(double angle) {
        passThruServoR.turnToAngle(angle);
        passThruServoL.turnToAngle(angle);
    }

    /**
     * Hold main passthrough servo positions
     */
    public void runPassThruServos () {
        switch (currentPassThruPos) {
            case FRONT_IDLE:
                if (clawIsTilted) currentPassThruPos = passThruPos.FRONT;
                break;
            case FRONT:
                if (!clawIsTilted) {
                    setPassThruAngle(RobotConfig.PASS_FRONT_ANGLE);
                    currentPassThruPos = passThruPos.FRONT_IDLE;
                } else setPassThruAngle(RobotConfig.PASS_FRONT_TILT_ANGLE);
                break;
            case PIVOT_POS:
                setPassThruAngle(RobotConfig.PASS_PIVOT_ANGLE);
                break;
            case BACK_IDLE:
                if (clawIsTilted) currentPassThruPos = passThruPos.BACK;
                break;
            case BACK:
                if (!clawIsTilted) {
                    setPassThruAngle(RobotConfig.PASS_BACK_ANGLE);
                    currentPassThruPos = passThruPos.BACK_IDLE;
                } else setPassThruAngle(RobotConfig.PASS_BACK_TILT_ANGLE);
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
    public void runPassThruStates () {
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
     * Named lift position
     */
    public enum liftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    /**
     * Set target for lift motion profile
     * @param height Desired named position to run to
     */
    public void setTargetLiftPos (liftPos height) {
        clawIsTilted = false;
        switch (height){
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
                clawIsTilted = true;
                targetLiftPos = RobotConfig.HEIGHT_LOW;
                targetLiftPosName = liftPos.LOW.name();
                break;
            case MED:
                clawIsTilted = true;
                targetLiftPos = RobotConfig.HEIGHT_MEDIUM;
                targetLiftPosName = liftPos.MED.name();
                break;
            case TALL:
                clawIsTilted = true;
                targetLiftPos = RobotConfig.HEIGHT_TALL;
                targetLiftPosName = liftPos.TALL.name();
                break;
        }
        updateLiftProfile();
    }

    /**
     * Set target for lift motion profile
     * @param height Desired position (in inches) to run to
     */
    public void setTargetLiftPos (double height) {
        targetLiftPos = height;
        targetLiftPosName = Double.toString(height);
        updateLiftProfile();
    }

    /**
     * Sets the target lift state to the current lift state
     * Internal acknowledgement method, use if disabling lift PIDF, moving, and then re-enabling lift PIDF.
     */
    public void setLiftStateToCurrent() {
        liftProfileState = new MotionState(currentLiftPos, currentLiftVelo, currentLiftAccel, currentLiftJerk);
        setTargetLiftPos(currentLiftPos);
    }

    /**
     * Update lift motion profile with a new target position
     */
    private void updateLiftProfile () {
//        UNCOMMENT IF NULL POINTER EXCEPTIONS ARE THROWN
//        if (targetLiftPos == currentLiftPos) targetLiftPos += 0.25;
        boolean goingDown = targetLiftPos < currentLiftPos;

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentLiftPos, currentLiftVelo, currentLiftAccel, currentLiftJerk),
                new MotionState(targetLiftPos, 0, 0, 0),
                goingDown? RobotConfig.LIFT_MAX_DOWN_VELO: RobotConfig.LIFT_MAX_UP_VELO,
                goingDown? RobotConfig.LIFT_MAX_DOWN_ACCEL: RobotConfig.LIFT_MAX_UP_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );

        liftProfileTimer.reset();
    }

    /**
     * Update lift PIDF controller gains with constants from RobotConfig.java
     */
    private void updateLiftGains () {
        liftController.setGains(
                new PIDCoefficients(
                        RobotConfig.LIFT_kP,
                        RobotConfig.LIFT_kI,
                        RobotConfig.LIFT_kD
                ),
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
     * Calculates time derivative for velocity, acceleration, and jerk
     */
    public void readLiftPos () {
        double
                lastLiftPos = currentLiftPos,
                lastLiftVelo = currentLiftVelo,
                lastLiftAccel = currentLiftAccel,
                currentTimeStamp = liftDerivTimer.seconds(),
                dt = currentTimeStamp - lastTimestamp;
        boolean dtIsZero = dt == 0.0;
        lastTimestamp = currentTimeStamp;

        veloFilter.setGains(RobotConfig.LIFT_VELO_FILTER_GAIN, RobotConfig.LIFT_VELO_ESTIMATE_COUNT);
        accelFilter.setGains(RobotConfig.LIFT_ACCEL_FILTER_GAIN, RobotConfig.LIFT_ACCEL_ESTIMATE_COUNT);
        jerkFilter.setGains(RobotConfig.LIFT_JERK_FILTER_GAIN, RobotConfig.LIFT_JERK_ESTIMATE_COUNT);

        currentLiftPos = lift_motor2.encoder.getPosition() * RobotConfig.LIFT_INCHES_PER_TICK;
        currentLiftVelo = dtIsZero? 0.0: (veloFilter.getEstimate((currentLiftPos - lastLiftPos) / dt));
        currentLiftAccel = dtIsZero? 0.0: (accelFilter.getEstimate((currentLiftVelo - lastLiftVelo) / dt));
        currentLiftJerk = dtIsZero? 0.0: (jerkFilter.getEstimate((currentLiftAccel - lastLiftAccel) / dt));
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift () {
        jerkFilter.resetPastValues();
        accelFilter.resetPastValues();
        veloFilter.resetPastValues();

        lastTimestamp = 0.0;
        currentLiftJerk = 0.0;
        currentLiftAccel = 0.0;
        currentLiftVelo = 0.0;
        currentLiftPos = 0.0;
        targetLiftPos = 0.0;
        targetLiftPosName = liftPos.FLOOR.name();
        clawIsTilted = false;

        liftDerivTimer.reset();
        liftProfileTimer.reset();
        liftController.reset();
        lift_motor2.resetEncoder();

        liftProfileState = new MotionState(0.0, 0.0, 0.0, 0.0);

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                liftProfileState,
                liftProfileState,
                RobotConfig.LIFT_MAX_UP_VELO,
                RobotConfig.LIFT_MAX_UP_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );
    }

    /**
     * Runs lift PIDF controller to track along motion profile
     */
    public void runLiftToPos () {
        liftProfileState = liftProfile.get(liftProfileTimer.seconds());

        liftController.setTargetPosition(liftProfileState.getX());
        liftController.setTargetVelocity(liftProfileState.getV());
        liftController.setTargetAcceleration(liftProfileState.getA());

        if (liftController.atTargetPosition(currentLiftPos)) liftController.reset();

        updateLiftGains();
        runLift(liftController.update(currentLiftPos));
    }

    /**
     * Run lift motors
     * @param veloCommand Pass in a velocity between 0 and 1
     */
    public void runLift (double veloCommand) {
        veloCommand += getLiftGravityFF();
        lift_motor1.set(veloCommand);
        lift_motor2.set(veloCommand);
        lift_motor3.set(veloCommand);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     * @return Velocity command for lift
     */
    private double getLiftGravityFF () {
        double veloCommand = 0.0;

        if      (currentLiftPos >= RobotConfig.STAGES_FOUR)         veloCommand = RobotConfig.LIFT_kG_FOUR;
        else if (currentLiftPos >= RobotConfig.STAGES_THREE)        veloCommand = RobotConfig.LIFT_kG_THREE;
        else if (currentLiftPos >= RobotConfig.STAGES_TWO)          veloCommand = RobotConfig.LIFT_kG_TWO;
        else if (currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE)   veloCommand = RobotConfig.LIFT_kG_ONE;

        return veloCommand;
    }

    public void toggleClawTilt() {
        clawIsTilted = !clawIsTilted;
    }

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    /**
     * Closes and lift claw if open
     * Opens and lowers claw if already closed
     */
    public void triggerClaw () {
        if (clawIsOpen) liftClaw(); else dropClaw();
    }

    /**
     * Set state of the claw
     * @param open True if open; false if closed
     */
    public void setClawOpen (boolean open) {
        clawIsOpen = open;
    }

    /**
     * Holds claw servo position
     */
    public void runClaw () {
        clawServo.turnToAngle(
                clawIsOpen?
                    passThruIsMoving? // open
                            RobotConfig.CLAW_PASS_ANGLE: // moving
                            RobotConfig.CLAW_OPEN_ANGLE: // not moving
                    RobotConfig.CLAW_CLOSED_ANGLE // closed
        );

        if ((liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) && !clawHasLifted) {
            raiseClaw();
            clawHasLifted = true;
        }
    }

    /**
     * Lifts claw either:
     *      6 inches if grabbing off stack
     *      2 inches if grabbing off the floor
     */
    public void raiseClaw () {
        setTargetLiftPos(Math.min(
                currentLiftPos + ((currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE)? 6: 2),
                RobotConfig.HEIGHT_TALL
        ));
    }

    /**
     * Closes claw
     * Waits for claw to close
     * Lifts claw
     */
    public void liftClaw () {
        setClawOpen(false);
        clawHasLifted = false;
        liftClawTimer.reset();
    }

    /**
     * Opens claw and runs lift to floor position
     */
    public void dropClaw () {
        dropClaw(liftPos.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     * @param height Named position to run lift to
     */
    public void dropClaw (liftPos height) {
        setClawOpen(true);
        setTargetLiftPos(height);
    }

    public void togglePivot () {
        pivotIsFront = !pivotIsFront;
    }

    /**
     * Holds pivot servo position
     */
    public void runPivot () {
        pivotServo.turnToAngle(pivotIsFront? RobotConfig.PIVOT_FRONT_ANGLE: RobotConfig.PIVOT_BACK_ANGLE);
    }

    /**
     * Activates automated passthrough sequence
     */
    public void triggerPassThru () {
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
    public void togglePassThru () {
        if (currentPassThruState != passThruState.FRONT) {
            currentPassThruPos = passThruPos.FRONT;
            currentPassThruState = passThruState.FRONT;
        } else {
            currentPassThruPos = passThruPos.BACK;
            currentPassThruState = passThruState.BACK;
        }
    }

    /**
     * Holds cone arm servos in position
     * @param down True if arms are to be down; false if arms should be upright
     */
    public void runConeArms (boolean down) {
        double angle = down? RobotConfig.ARM_DOWN_ANGLE : RobotConfig.ARM_UP_ANGLE;
        coneArmServoL.turnToAngle(280.0 - angle);
        coneArmServoR.turnToAngle(angle);
    }

    /**
     * Print relevant telemetry of the system (particularly lift data)
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry (MultipleTelemetry telemetry) {
        telemetry.addData("Lift current position (in)", currentLiftPos);
        telemetry.addData("Lift profile position (in)", liftProfileState.getX());
        telemetry.addData("Lift target position (name)", targetLiftPosName);
        telemetry.addLine();
        telemetry.addData("Lift position error (in)", liftController.getCurrentFilterEstimate());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentLiftVelo);
        telemetry.addData("Lift profile velocity (in/s)", liftProfileState.getV());
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentLiftAccel);
        telemetry.addLine();
        telemetry.addData("Lift current jerk (in/s^3)", currentLiftJerk);
    }
}
