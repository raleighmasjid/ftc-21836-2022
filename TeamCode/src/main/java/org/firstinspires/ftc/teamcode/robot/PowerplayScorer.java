package org.firstinspires.ftc.teamcode.robot;


import androidx.annotation.NonNull;

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
     * Motor powering the dual lift system
     */
    private MotorEx
            lift_motor1,
            lift_motor2,
            lift_motor3;
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
     * Timer for tracking along lift motion profile
     */
    private ElapsedTime liftProfileTimer;
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
    private ElapsedTime liftDerivTimer;
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
    private double currentTimestamp;
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
     * Timer to track sequential passthrough events
     */
    private static ElapsedTime passThruTimer;
    /**
     * Count of ticks per revolution for lift motors
     */
    private static final double LIFT_TICKS = 145.1;
    /**
     * Revolutions per minute for lift motors
     */
    private static final double LIFT_RPM = 1150;

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
        clawIsOpen = true;
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
        passThruServoL.turnToAngle(355.0 - angle);
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
                    setPassThruAngle(RobotConfig.ANGLE_PASS_FRONT);
                    currentPassThruPos = passThruPos.FRONT_IDLE;
                } else setPassThruAngle(RobotConfig.ANGLE_PASS_FRONT + RobotConfig.ANGLE_PASS_TILT);
                break;
            case PIVOT_POS:
                setPassThruAngle(RobotConfig.ANGLE_PASS_PIVOT);
                break;
            case BACK_IDLE:
                if (clawIsTilted) currentPassThruPos = passThruPos.BACK;
                break;
            case BACK:
                if (!clawIsTilted) {
                    setPassThruAngle(RobotConfig.ANGLE_PASS_BACK);
                    currentPassThruPos = passThruPos.BACK_IDLE;
                } else setPassThruAngle(RobotConfig.ANGLE_PASS_BACK - RobotConfig.ANGLE_PASS_TILT);
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
     * Named lift position
     */
    public enum liftPos {
        FLOOR,
        TWO,
        THREE,
        FOUR,
        FIVE,
        LOW,
        MED,
        TALL
    }

    /**
     * Set target for lift motion profile
     * @param height Desired named position to run to
     */
    public void setTargetLiftPos (@NonNull liftPos height) {
        clawIsTilted = false;
        double targetLiftPos;
        switch (height){
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
            case FLOOR:
            default:
                targetLiftPos = RobotConfig.HEIGHT_FLOOR;
                targetLiftPosName = liftPos.FLOOR.name();
                break;
        }
        updateLiftProfile(targetLiftPos);
    }

    /**
     * Set target for lift motion profile
     * @param targetLiftPos Desired position (in inches) to run to
     */
    public void setTargetLiftPos (double targetLiftPos) {
        targetLiftPosName = Double.toString(targetLiftPos);
        updateLiftProfile(targetLiftPos);
    }

    /**
     * Sets the target lift state to the current lift state
     * Internal acknowledgement method, use if disabling lift PIDF, moving, and then re-enabling lift PIDF.
     */
    public void setLiftStateToCurrent() {
        profileLiftState = new MotionState(currentLiftState.getX(), currentLiftState.getV(), currentLiftState.getA(), currentLiftState.getJ());
        setTargetLiftPos(currentLiftState.getX());
    }

    /**
     * Update lift motion profile with a new target position
     */
    private void updateLiftProfile (double targetLiftPos) {
        boolean goingDown = targetLiftPos < currentLiftState.getX();

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                currentLiftState,
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
     * Calculates velocity, acceleration, and jerk
     * Saves readings to currentLiftState
     */
    public void readLiftPos () {
        double
                newTimestamp = liftDerivTimer.seconds(),
                dt = newTimestamp - currentTimestamp;
        boolean dtIsZero = dt == 0.0;
        currentTimestamp = newTimestamp;

        veloFilter.setGains(RobotConfig.LIFT_VELO_FILTER_GAIN, RobotConfig.LIFT_VELO_ESTIMATE_COUNT);
        accelFilter.setGains(RobotConfig.LIFT_ACCEL_FILTER_GAIN, RobotConfig.LIFT_ACCEL_ESTIMATE_COUNT);
        jerkFilter.setGains(RobotConfig.LIFT_JERK_FILTER_GAIN, RobotConfig.LIFT_JERK_ESTIMATE_COUNT);

        double newLiftPos = lift_motor2.encoder.getPosition() * RobotConfig.LIFT_INCHES_PER_TICK;
        double newLiftVelo = dtIsZero? 0.0: (veloFilter.getEstimate((newLiftPos - currentLiftState.getX()) / dt));
        double newLiftAccel = dtIsZero? 0.0: (accelFilter.getEstimate((newLiftVelo - currentLiftState.getV()) / dt));
        double newLiftJerk = dtIsZero? 0.0: (jerkFilter.getEstimate((newLiftAccel - currentLiftState.getA()) / dt));

        currentLiftState = new MotionState(newLiftPos, newLiftVelo, newLiftAccel, newLiftJerk);
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift () {
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
    public void runLiftToPos () {
        profileLiftState = liftProfile.get(liftProfileTimer.seconds());

        liftController.setTargetPosition(profileLiftState.getX());
        liftController.setTargetVelocity(profileLiftState.getV());
        liftController.setTargetAcceleration(profileLiftState.getA());

        if (liftController.atTargetPosition(currentLiftState.getX())) liftController.reset();

        updateLiftGains();
        runLift(liftController.update(currentLiftState.getX()));
    }

    /**
     * Run lift motors
     * @param veloCommand Pass in a velocity between 0 and 1
     */
    public void runLift (double veloCommand) {
        double commandWithkG = veloCommand + getLiftGravityFF();
        lift_motor1.set(commandWithkG);
        lift_motor2.set(commandWithkG);
        lift_motor3.set(commandWithkG);
    }

    /**
     * Calculates anti-gravity feedforward for a 4-stage continuous rigged linear slide system
     * @return Velocity command for lift
     */
    private double getLiftGravityFF () {
        double veloCommand;

        if      (currentLiftState.getX() >= RobotConfig.STAGES_FOUR)         veloCommand = RobotConfig.LIFT_kG_FOUR;
        else if (currentLiftState.getX() >= RobotConfig.STAGES_THREE)        veloCommand = RobotConfig.LIFT_kG_THREE;
        else if (currentLiftState.getX() >= RobotConfig.STAGES_TWO)          veloCommand = RobotConfig.LIFT_kG_TWO;
        else if (currentLiftState.getX() > RobotConfig.LIFT_POS_TOLERANCE)   veloCommand = RobotConfig.LIFT_kG_ONE;
        else    veloCommand = 0.0;

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
        if (clawIsOpen) grabCone(); else dropCone();
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
                            RobotConfig.ANGLE_CLAW_PASS : // moving
                            RobotConfig.ANGLE_CLAW_OPEN : // not moving
                    RobotConfig.ANGLE_CLAW_CLOSED // closed
        );

        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
    }

    /**
     * Lifts claw either:
     *      6 inches if grabbing off stack
     *      2 inches if grabbing off the floor
     */
    public void liftClaw () {
        setTargetLiftPos(currentLiftState.getX() + ((currentLiftState.getX() > RobotConfig.LIFT_POS_TOLERANCE)? 6: 2));
        clawHasLifted = true;
    }

    /**
     * Closes claw
     * Waits for claw to close
     * Lifts claw
     */
    public void grabCone () {
        setClawOpen(false);
        if (currentLiftState.getX() <= (RobotConfig.HEIGHT_FIVE + RobotConfig.LIFT_POS_TOLERANCE)) {
            clawHasLifted = false;
            liftClawTimer.reset();
        }
    }

    /**
     * Opens claw and runs lift to floor position
     */
    public void dropCone () {
        dropCone(liftPos.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     * @param height Named position to run lift to
     */
    public void dropCone (liftPos height) {
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
        pivotServo.turnToAngle(355.0 - (pivotIsFront? RobotConfig.ANGLE_PIVOT_FRONT: RobotConfig.ANGLE_PIVOT_BACK));
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
        currentPassThruPos = passThruInFront? passThruPos.BACK: passThruPos.FRONT;
        currentPassThruState = passThruInFront? passThruState.BACK: passThruState.FRONT;
        passThruInFront = !passThruInFront;
    }

    /**
     * Holds cone arm servos in position
     * @param down True if arms are to be down; false if arms should be upright
     */
    public void runConeArms (boolean down) {
        double angle = down? RobotConfig.ANGLE_ARM_DOWN : RobotConfig.ANGLE_ARM_UP;
        coneArmServoL.turnToAngle(280.0 - angle);
        coneArmServoR.turnToAngle(angle);
    }

    /**
     * Print relevant telemetry of the system (particularly lift data)
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry (MultipleTelemetry telemetry) {
        telemetry.addData("Lift encoder reading (ticks)", lift_motor2.encoder.getPosition());
        telemetry.addData("Lift current position (in)", currentLiftState.getX());
        telemetry.addData("Lift profile position (in)", profileLiftState.getX());
        telemetry.addData("Lift target position (name)", targetLiftPosName);
        telemetry.addLine();
        telemetry.addData("Lift position error (in)", liftController.getCurrentFilterEstimate());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentLiftState.getV());
        telemetry.addData("Lift profile velocity (in/s)", profileLiftState.getV());
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentLiftState.getA());
        telemetry.addLine();
        telemetry.addData("Lift current jerk (in/s^3)", currentLiftState.getJ());
    }
}
