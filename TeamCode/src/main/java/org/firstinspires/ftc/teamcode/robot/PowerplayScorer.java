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

import org.firstinspires.ftc.teamcode.control.controller.PIDFController;
import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter;
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
     * PID controller for lift
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
    private double currentLiftPos, currentLiftVelo, currentLiftAccel, currentLiftJerk;
    /**
     * Named end target lift position
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
     * Timer to track sequential passthrough events
     */
    private static ElapsedTime passThruTimer;
    /**
     * End target lift position
     */
    private double targetLiftPos;

    /**
     * Named lift position
     */
    public enum LiftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    private final IIRLowPassFilter jerkFilter, accelFilter, veloFilter;
    private double currentTimestamp;
    private boolean passThruIsMoving;
    private boolean passThruInFront;
    private double coneArmsAngle;
    private boolean clawIsOpen;
    private boolean clawIsTilted;

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
                RobotConfig.LIFT_PID_FILTER_GAIN,
                RobotConfig.LIFT_UP_kV,
                RobotConfig.LIFT_UP_kA,
                RobotConfig.LIFT_UP_kS,
                RobotConfig.LIFT_INTEGRATION_MAX_VELO
        );
        liftController.setOutputBounds(-1.0, 1.0);

        veloFilter = new IIRLowPassFilter();
        accelFilter = new IIRLowPassFilter();
        jerkFilter = new IIRLowPassFilter();

        veloFilter.setGain(RobotConfig.LIFT_VELO_FILTER_GAIN);
        accelFilter.setGain(RobotConfig.LIFT_ACCEL_FILTER_GAIN);
        jerkFilter.setGain(RobotConfig.LIFT_JERK_FILTER_GAIN);

        clawHasLifted = true;
        pivotIsFront = true;
        passThruInFront = true;
        passThruIsMoving = false;
        clawIsOpen = true;
        clawIsTilted = false;
        coneArmsAngle = 0.0;

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
     * Set target for lift motion profile
     *
     * @param height Desired named position to run to
     */
    public void setTargetLiftPos(@NonNull LiftPos height) {
        clawIsTilted = height == LiftPos.LOW || height == LiftPos.MED || height == LiftPos.TALL;
        switch (height) {
            case TWO:
                targetLiftPos = RobotConfig.HEIGHT_TWO;
                targetLiftPosName = LiftPos.TWO.name();
                break;
            case THREE:
                targetLiftPos = RobotConfig.HEIGHT_THREE;
                targetLiftPosName = LiftPos.THREE.name();
                break;
            case FOUR:
                targetLiftPos = RobotConfig.HEIGHT_FOUR;
                targetLiftPosName = LiftPos.FOUR.name();
                break;
            case FIVE:
                targetLiftPos = RobotConfig.HEIGHT_FIVE;
                targetLiftPosName = LiftPos.FIVE.name();
                break;
            case LOW:
                targetLiftPos = RobotConfig.HEIGHT_LOW;
                targetLiftPosName = LiftPos.LOW.name();
                break;
            case MED:
                targetLiftPos = RobotConfig.HEIGHT_MEDIUM;
                targetLiftPosName = LiftPos.MED.name();
                break;
            case TALL:
                targetLiftPos = RobotConfig.HEIGHT_TALL;
                targetLiftPosName = LiftPos.TALL.name();
                break;
            default:
                targetLiftPos = RobotConfig.HEIGHT_FLOOR;
                targetLiftPosName = LiftPos.FLOOR.name();
                break;
        }
        updateLiftProfile();
    }

    /**
     * Set target for lift motion profile
     *
     * @param targetLiftPos Desired position (in inches) to run to
     */
    public void setTargetLiftPos(double targetLiftPos) {
        targetLiftPosName = Double.toString(targetLiftPos);
        updateLiftProfile();
        this.targetLiftPos = targetLiftPos;
    }

    /**
     * Sets the target lift state to the current lift state
     */
    public void setLiftStateToCurrent() {
        setTargetLiftPos(currentLiftPos);
    }

    /**
     * Update lift motion profile with a new target position
     */
    private void updateLiftProfile() {
        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentLiftPos, currentLiftVelo, currentLiftAccel, currentLiftJerk),
                new MotionState(targetLiftPos, 0.0, 0.0, 0.0),
                RobotConfig.LIFT_MAX_VELO,
                RobotConfig.LIFT_MAX_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );

        liftProfileTimer.reset();
        updateLiftGains();
    }

    /**
     * Update lift PIDF controller gains with constants from RobotConfig.java
     */
    private void updateLiftGains() {
        boolean goingDown = targetLiftPos < currentLiftPos;

        liftController.getPid().setGains(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_PID_FILTER_GAIN
        );
        liftController.getFeedforward().setGains(
                goingDown ? RobotConfig.LIFT_DOWN_kV : RobotConfig.LIFT_UP_kV,
                goingDown ? RobotConfig.LIFT_DOWN_kA : RobotConfig.LIFT_UP_kA,
                goingDown ? RobotConfig.LIFT_DOWN_kS : RobotConfig.LIFT_UP_kS
        );
        liftController.setMaxIntegrationVelocity(RobotConfig.LIFT_INTEGRATION_MAX_VELO);
    }

    /**
     * Reads lift encoder value and converts to position in inches
     * Calculates velocity, acceleration, and jerk
     * Saves readings to currentLift____
     */
    public void readLiftPos() {
        double lastLiftPos = currentLiftPos,
                lastLiftVelo = currentLiftVelo,
                lastLiftAccel = currentLiftAccel,
                lastTimestamp = currentTimestamp;
        currentTimestamp = liftDerivTimer.seconds();
        double dt = currentTimestamp - lastTimestamp;
        boolean dtIsZero = dt == 0.0;

        veloFilter.setGain(RobotConfig.LIFT_VELO_FILTER_GAIN);
        accelFilter.setGain(RobotConfig.LIFT_ACCEL_FILTER_GAIN);
        jerkFilter.setGain(RobotConfig.LIFT_JERK_FILTER_GAIN);

        currentLiftPos = lift_motor2.encoder.getPosition() * RobotConfig.LIFT_INCHES_PER_TICK;
        currentLiftVelo = dtIsZero ? 0.0 : (veloFilter.getEstimate((currentLiftPos - lastLiftPos) / dt));
        currentLiftAccel = dtIsZero ? 0.0 : (accelFilter.getEstimate((currentLiftVelo - lastLiftVelo) / dt));
        currentLiftJerk = dtIsZero ? 0.0 : (jerkFilter.getEstimate((currentLiftAccel - lastLiftAccel) / dt));
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        jerkFilter.clearMemory();
        accelFilter.clearMemory();
        veloFilter.clearMemory();

        currentTimestamp = 0.0;
        targetLiftPosName = LiftPos.FLOOR.name();
        clawIsTilted = false;

        liftDerivTimer.reset();
        liftProfileTimer.reset();
        liftController.getPid().resetIntegral();
        lift_motor2.resetEncoder();

        currentLiftPos = 0.0;
        currentLiftVelo = 0.0;
        currentLiftAccel = 0.0;
        currentLiftJerk = 0.0;

        targetLiftPos = 0.0;
        profileLiftState = new MotionState(0.0, 0.0, 0.0, 0.0);

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0.0, 0.0, 0.0, 0.0),
                new MotionState(0.0, 0.0, 0.0, 0.0),
                RobotConfig.LIFT_MAX_VELO,
                RobotConfig.LIFT_MAX_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );
    }

    /**
     * Runs lift PIDF controller to track along motion profile
     */
    public void runLiftToPos() {
        profileLiftState = liftProfile.get(liftProfileTimer.seconds());

        liftController.getPid().setTargetPosition(profileLiftState.getX());
        liftController.getFeedforward().setTargetVelocity(profileLiftState.getV());
        liftController.getFeedforward().setTargetAcceleration(profileLiftState.getA());

        if (Math.abs(targetLiftPos - currentLiftPos) <= RobotConfig.LIFT_POS_TOLERANCE) {
            liftController.getPid().resetIntegral();
        }

        runLift(liftController.update(currentLiftPos));
    }

    /**
     * Run lift motors
     *
     * @param veloCommand Pass in a velocity command between 0 and 1
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
     * Closes claw
     * Waits for claw to close
     * Lifts claw
     */
    public void grabCone() {
        setClawOpen(false);
        if (currentLiftPos <= (RobotConfig.HEIGHT_FIVE + RobotConfig.LIFT_POS_TOLERANCE)) {
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
        setTargetLiftPos(currentLiftPos + ((currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE) ? 6 : 2));
        clawHasLifted = true;
    }

    /**
     * Opens claw and runs lift to floor position
     */
    public void dropCone() {
        dropCone(LiftPos.FLOOR);
    }

    /**
     * Opens claw and runs lift to named position
     *
     * @param height Named position to run lift to
     */
    public void dropCone(LiftPos height) {
        setClawOpen(true);
        setTargetLiftPos(height);
    }

    /**
     * Holds claw servo position
     */
    public void runClaw() {
        clawServo.turnToAngle(
                !clawIsOpen || passThruIsMoving ?
                        RobotConfig.ANGLE_CLAW_CLOSED :
                        RobotConfig.ANGLE_CLAW_OPEN
        );

        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
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
     * Activates passthrough
     */
    public void triggerPassThru() {
        togglePassThru();
        togglePivot();
        passThruIsMoving = true;
        passThruTimer.reset();
    }

    /**
     * Toggles position of main passthrough servos
     */
    public void togglePassThru() {
        passThruInFront = !passThruInFront;
    }

    /**
     * Hold main passthrough servo positions
     */
    public void runPassThru() {
        double tiltOffset = clawIsTilted ? RobotConfig.ANGLE_PASS_TILT : 0.0;
        double angle =
                passThruInFront ?
                        RobotConfig.ANGLE_PASS_FRONT + tiltOffset :
                        RobotConfig.ANGLE_PASS_BACK - tiltOffset;

        passThruServoR.turnToAngle(angle);
        passThruServoL.turnToAngle(355.0 - angle);

        if (passThruIsMoving && passThruTimer.seconds() >= (clawIsTilted ? RobotConfig.TIME_PASS_TILTED : RobotConfig.TIME_PASS)) {
            passThruIsMoving = false;
        }
    }

    /**
     * Holds cone arm servos in position
     */
    public void runConeArms(double angle) {
        coneArmsAngle = angle * RobotConfig.ANGLE_ARM_DOWN;
        coneArmServoL.turnToAngle(280.0 - coneArmsAngle);
        coneArmServoR.turnToAngle(coneArmsAngle);
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
        telemetry.addData("Lift current position (in)", currentLiftPos);
        telemetry.addData("Lift profile position (in)", profileLiftState.getX());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentLiftVelo);
        telemetry.addData("Lift profile velocity (in/s)", profileLiftState.getV());
        telemetry.addData("Lift error derivative (in/s)", liftController.getPid().getErrorDeriv());
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentLiftAccel);
        telemetry.addLine();
        telemetry.addData("Lift current jerk (in/s^3)", currentLiftJerk);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Cone arms angle", coneArmsAngle);
        telemetry.addLine();
        telemetry.addData("Claw is", clawIsOpen ? "open" : "closed");
        telemetry.addLine();
        telemetry.addData("Pivot is oriented to", pivotIsFront ? "front" : "back");
        telemetry.addLine();
        telemetry.addData("Passthrough is in the", passThruInFront ? "front" : "back");
    }
}
