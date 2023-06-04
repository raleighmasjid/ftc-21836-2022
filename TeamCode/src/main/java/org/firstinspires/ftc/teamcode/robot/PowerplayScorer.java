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
 * Contains a 3-motor motion profiled lift, multi-function claw, and motion profiled passthrough
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

    private final ElapsedTime passThruProfileTimer, liftProfileTimer, liftDerivTimer, liftClawTimer;

    private final IIRLowPassFilter accelFilter, veloFilter;

    /**
     * PIDF controller for lift
     */
    private final PIDFController liftController;

    private MotionProfile passThruProfile, liftProfile;

    /**
     * Lift state grabbed from motion profile
     */
    private MotionState profileLiftState;

    private String targetLiftPosName;

    private double currentPassThruAngle, currentPassThruVelo, currentLiftPos, currentLiftVelo, currentLiftAccel, targetLiftPos;

    private boolean clawHasLifted, pivotIsFront, passThruInFront, passThruTriggered, clawIsOpen, clawIsTilted;

    /**
     * Named lift position
     */
    public enum LiftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

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
                RobotConfig.LIFT_kD_FILTER_GAIN,
                RobotConfig.LIFT_UP_kV,
                RobotConfig.LIFT_UP_kA,
                RobotConfig.LIFT_UP_kS,
                RobotConfig.LIFT_INTEGRATION_MAX_VELO
        );
        liftController.setOutputBounds(-1.0, 1.0);

        veloFilter = new IIRLowPassFilter(RobotConfig.LIFT_VELO_FILTER_GAIN);
        accelFilter = new IIRLowPassFilter(RobotConfig.LIFT_ACCEL_FILTER_GAIN);

        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();
        liftProfileTimer = new ElapsedTime();
        liftProfileTimer.reset();
        liftDerivTimer = new ElapsedTime();
        liftDerivTimer.reset();
        passThruProfileTimer = new ElapsedTime();
        passThruProfileTimer.reset();

        clawHasLifted = true;
        setPivotIsFront(true);
        passThruInFront = true;
        clawIsOpen = true;
        setClawTilt(false);
        passThruTriggered = false;

        currentPassThruAngle = RobotConfig.ANGLE_PASS_FRONT;
        currentPassThruVelo = 0.0;
        updatePassThruProfile();

        resetLift();
    }

    /**
     * Reads lift encoder value and converts to position in inches
     * Calculates velocity, acceleration
     * Saves readings to {@link #currentLiftPos}, {@link #currentLiftVelo}, and {@link #currentLiftAccel}
     */
    public void readLiftPos() {
        double lastLiftPos = currentLiftPos,
                lastLiftVelo = currentLiftVelo,
                timerSeconds = liftDerivTimer.seconds(),
                dt = timerSeconds == 0 ? 0.002 : timerSeconds;
        liftDerivTimer.reset();
        updateLiftGains();

        currentLiftPos = lift_motor2.encoder.getPosition() * RobotConfig.LIFT_INCHES_PER_TICK;
        currentLiftVelo = veloFilter.getEstimate((currentLiftPos - lastLiftPos) / dt);
        currentLiftAccel = accelFilter.getEstimate((currentLiftVelo - lastLiftVelo) / dt);
    }

    /**
     * Update lift PIDF controller gains with constants from RobotConfig.java
     */
    private void updateLiftGains() {
        boolean goingDown = targetLiftPos < currentLiftPos;

        veloFilter.setGain(RobotConfig.LIFT_VELO_FILTER_GAIN);
        accelFilter.setGain(RobotConfig.LIFT_ACCEL_FILTER_GAIN);

        liftController.PID.setGains(
                RobotConfig.LIFT_kP,
                RobotConfig.LIFT_kI,
                RobotConfig.LIFT_kD,
                RobotConfig.LIFT_kD_FILTER_GAIN
        );
        liftController.Feedforward.setGains(
                goingDown ? RobotConfig.LIFT_DOWN_kV : RobotConfig.LIFT_UP_kV,
                goingDown ? RobotConfig.LIFT_DOWN_kA : RobotConfig.LIFT_UP_kA,
                goingDown ? RobotConfig.LIFT_DOWN_kS : RobotConfig.LIFT_UP_kS
        );
        liftController.setMaxIntegrationVelocity(RobotConfig.LIFT_INTEGRATION_MAX_VELO);
    }

    /**
     * Sets the target lift state to the current lift state
     */
    public void setLiftStateToCurrent() {
        setTargetLiftPos(currentLiftPos);
    }

    private double getConesHeight(int numOfCones) {
        return (numOfCones - 1) * RobotConfig.HEIGHT_2 + RobotConfig.HEIGHT_FLOOR;
    }

    /**
     * Set target for lift motion profile
     *
     * @param height Desired named position to run to
     */
    public void setTargetLiftPos(@NonNull LiftPos height) {
        setClawTilt(height == LiftPos.LOW || height == LiftPos.MED || height == LiftPos.TALL);
        switch (height) {
            case TALL:
                targetLiftPos = RobotConfig.HEIGHT_TALL;
                targetLiftPosName = LiftPos.TALL.name();
                break;
            case MED:
                targetLiftPos = RobotConfig.HEIGHT_MEDIUM;
                targetLiftPosName = LiftPos.MED.name();
                break;
            case LOW:
                targetLiftPos = RobotConfig.HEIGHT_LOW;
                targetLiftPosName = LiftPos.LOW.name();
                break;
            case FIVE:
                targetLiftPos = getConesHeight(5);
                targetLiftPosName = LiftPos.FIVE.name();
                break;
            case FOUR:
                targetLiftPos = getConesHeight(4);
                targetLiftPosName = LiftPos.FOUR.name();
                break;
            case THREE:
                targetLiftPos = getConesHeight(3);
                targetLiftPosName = LiftPos.THREE.name();
                break;
            case TWO:
                targetLiftPos = getConesHeight(2);
                targetLiftPosName = LiftPos.TWO.name();
                break;
            default:
            case FLOOR:
                targetLiftPos = getConesHeight(1);
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
        this.targetLiftPos = targetLiftPos;
        updateLiftProfile();
    }

    /**
     * Update lift motion profile with a new target position
     */
    private void updateLiftProfile() {
        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentLiftPos, currentLiftVelo),
                new MotionState(targetLiftPos, 0.0),
                RobotConfig.LIFT_MAX_VELO,
                RobotConfig.LIFT_MAX_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );
        liftProfileTimer.reset();
    }

    /**
     * Resets all internal lift variables
     */
    public void resetLift() {
        accelFilter.clearMemory();
        veloFilter.clearMemory();

        lift_motor2.resetEncoder();
        liftController.PID.resetIntegral();

        currentLiftPos = 0.0;
        currentLiftVelo = 0.0;
        currentLiftAccel = 0.0;

        targetLiftPos = 0.0;
        targetLiftPosName = LiftPos.FLOOR.name();
        setClawTilt(false);

        updateLiftProfile();
        profileLiftState = new MotionState(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Runs lift PIDF controller to track along motion profile
     */
    public void runLiftToPos() {
        profileLiftState = liftProfile.get(liftProfileTimer.seconds());

        liftController.PID.setTargetPosition(profileLiftState.getX());
        liftController.Feedforward.setTargetVelocity(profileLiftState.getV());
        liftController.Feedforward.setTargetAcceleration(profileLiftState.getA());

        if (targetLiftPos == currentLiftPos) liftController.PID.resetIntegral();

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
        return currentLiftPos >= RobotConfig.HEIGHT_STAGES_4 ? RobotConfig.LIFT_kG_4 :
                currentLiftPos >= RobotConfig.HEIGHT_STAGES_3 ? RobotConfig.LIFT_kG_3 :
                        currentLiftPos >= RobotConfig.HEIGHT_STAGES_2 ? RobotConfig.LIFT_kG_2 :
                                currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE ? RobotConfig.LIFT_kG_1 :
                                        0.0;
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
     * Waits for claw to close
     * Lifts claw
     */
    public void grabCone() {
        clawIsOpen = false;
        if (currentLiftPos <= (getConesHeight(5) + RobotConfig.LIFT_POS_TOLERANCE)) {
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
        clawIsOpen = true;
        setTargetLiftPos(height);
    }

    /**
     * Holds claw servo position
     */
    public void runClaw() {
        clawServo.turnToAngle(clawIsOpen ? RobotConfig.ANGLE_CLAW_OPEN : RobotConfig.ANGLE_CLAW_CLOSED);
        if (!clawHasLifted && liftClawTimer.seconds() >= RobotConfig.TIME_CLAW) liftClaw();
    }

    public void togglePivot() {
        setPivotIsFront(!pivotIsFront);
    }

    public void setPivotIsFront(boolean isFront) {
        pivotIsFront = isFront;
        updatePassThruProfile();
    }

    /**
     * Holds pivot servo position
     */
    public void runPivot() {
        pivotServo.turnToAngle(355.0 - (pivotIsFront ? RobotConfig.ANGLE_PIVOT_FRONT : RobotConfig.ANGLE_PIVOT_BACK));
    }

    public void toggleClawTilt() {
        setClawTilt(!clawIsTilted);
    }

    public void setClawTilt(boolean tilted) {
        clawIsTilted = tilted;
        updatePassThruProfile();
    }

    /**
     * Triggers the arm, pivot, and closes the claw momentarily
     */
    public void triggerPassThru() {
        passThruTriggered = true;
        togglePassThru();
    }

    /**
     * Toggles position of main passthrough servos
     */
    public void togglePassThru() {
        passThruInFront = !passThruInFront;
        updatePassThruProfile();
    }

    /**
     * Update lift motion profile with a new target position
     */
    private void updatePassThruProfile() {
        double tiltOffset =
                clawIsTilted ?
                        RobotConfig.ANGLE_PASS_TILT :
                        (!passThruTriggered) && (passThruInFront != pivotIsFront) ? RobotConfig.ANGLE_PASS_MINI_TILT : 0.0;

        double targetPassThruAngle =
                passThruInFront ?
                        RobotConfig.ANGLE_PASS_FRONT + tiltOffset :
                        RobotConfig.ANGLE_PASS_BACK - tiltOffset;

        passThruProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPassThruAngle, currentPassThruVelo),
                new MotionState(targetPassThruAngle, 0.0),
                RobotConfig.PASS_MAX_VELO,
                RobotConfig.PASS_MAX_ACCEL,
                RobotConfig.PASS_MAX_JERK
        );

        passThruProfileTimer.reset();
    }

    /**
     * Hold main passthrough servo positions
     */
    public void runPassThru() {
        MotionState state = passThruProfile.get(passThruProfileTimer.seconds());
        currentPassThruAngle = state.getX();
        currentPassThruVelo = state.getV();
        passThruServoR.turnToAngle(currentPassThruAngle);
        passThruServoL.turnToAngle(355.0 - currentPassThruAngle);
        if (passThruTriggered && Math.abs(RobotConfig.PASS_PIVOT_POS - currentPassThruAngle) <= RobotConfig.PASS_PIVOT_POS_TOLERANCE) {
            pivotIsFront = passThruInFront;
            passThruTriggered = false;
        }
    }

    /**
     * Holds cone arm servos in position
     *
     * @param angleR The angle to turn the right arm to
     * @param angleL The angle to turn the left arm to
     */
    public void runConeArms(double angleR, double angleL) {
        coneArmServoL.turnToAngle(280.0 - (angleL * RobotConfig.ANGLE_ARM_DOWN));
        coneArmServoR.turnToAngle(angleR * RobotConfig.ANGLE_ARM_DOWN);
    }

    /**
     * Print debugging telemetry for the lift
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printLiftTelemetry(@NonNull MultipleTelemetry telemetry) {
        telemetry.addData("Lift current position (in)", currentLiftPos);
        telemetry.addData("Lift profile position (in)", profileLiftState.getX());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentLiftVelo);
        telemetry.addData("Lift profile velocity (in/s)", profileLiftState.getV());
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentLiftAccel);
        telemetry.addLine();
        telemetry.addData("Lift error integral (in*s)", liftController.PID.getErrorSum());
        telemetry.addData("Lift error (in)", liftController.PID.getLastError());
        telemetry.addData("Lift error derivative (in/s)", liftController.PID.getErrorDeriv());
    }

    /**
     * Print relevant telemetry of the system
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(@NonNull MultipleTelemetry telemetry) {
        telemetry.addData("Named target lift position", targetLiftPosName);
        telemetry.addLine();
        telemetry.addData("Claw is", clawIsOpen ? "open" : "closed");
        telemetry.addLine();
        telemetry.addData("Pivot is oriented to", pivotIsFront ? "front" : "back");
        telemetry.addLine();
        telemetry.addData("Passthrough is", (clawIsTilted ? "tilted " : "") + "at the " + (passThruInFront ? "front" : "back"));
    }
}
