package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.autonomous.DriveConstants.MAX_RPM;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

    private MotorEx
            lift_motor1,
            lift_motor2,
            lift_motor3;
    private SimpleServo
            clawServo,
            pivotServo,
            passThruServoR,
            passThruServoL,
            coneArmR,
            coneArmL;
    private PIDFController liftController;
    private MotionProfile liftProfile;
    public MotionState liftState;
    private ElapsedTime
            liftProfileTimer,
            liftDerivTimer;
    private static ElapsedTime
            passThruTimer,
            liftClawTimer;
    public DigitalChannel
            limitSwitch,
            LED1red,
            LED1green,
            LED2red,
            LED2green;
    private LowPassFilter
            jerkFilter,
            accelFilter,
            veloFilter;
    private double
            lastTimestamp,
            currentLiftJerk,
            currentLiftAccel,
            currentLiftVelo,
            currentLiftPos,
            targetLiftPos;
    private String targetLiftPosName;
    private boolean
            clawIsOpen,
            clawHasLifted,
            clawIsTilted,
            pivotIsFront,
            passThruIsMoving,
            passThruSwitched,
            passThruInFront;

    public void init (HardwareMap hw) {

        clawServo = new SimpleServo(hw,"claw right",0,355);
        pivotServo = new SimpleServo(hw, "claw pivot",0,355);
        passThruServoR = new SimpleServo(hw, "passthrough 1",0,355);
        passThruServoL = new SimpleServo(hw, "passthrough 2",0,355);
        coneArmR = new SimpleServo(hw, "arm right",0,280);
        coneArmL = new SimpleServo(hw, "arm left",0,280);

        lift_motor1 = new MotorEx(hw, "lift motor 1", LIFT_TICKS, MAX_RPM);
        lift_motor2 = new MotorEx(hw, "lift motor 2", LIFT_TICKS, MAX_RPM);
        lift_motor3 = new MotorEx(hw, "lift motor 3", LIFT_TICKS, MAX_RPM);

        limitSwitch = hw.get(DigitalChannel.class, "limit switch");

        LED1red = hw.get(DigitalChannel.class, "LED1red");
        LED1green = hw.get(DigitalChannel.class, "LED1green");

        LED2red = hw.get(DigitalChannel.class, "LED2red");
        LED2green = hw.get(DigitalChannel.class, "LED2green");

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        LED1red.setMode(DigitalChannel.Mode.OUTPUT);
        LED1green.setMode(DigitalChannel.Mode.OUTPUT);
        LED2red.setMode(DigitalChannel.Mode.OUTPUT);
        LED2green.setMode(DigitalChannel.Mode.OUTPUT);

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

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0.0, 0.0, 0.0, 0.0),
                new MotionState(0.0, 0.0, 0.0, 0.0),
                RobotConfig.LIFT_MAX_UP_VELO,
                RobotConfig.LIFT_MAX_UP_ACCEL,
                RobotConfig.LIFT_MAX_JERK
        );

        liftState = liftProfile.get(0.0);

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

    //  lift motor encoder resolution (ticks):
    private static final double LIFT_TICKS = 145.1;

    private enum passThruState {
        START,
        FRONT,
        FRONT_PIVOT,
        PIVOTING,
        BACK_PIVOT,
        BACK
    }
    private enum passThruPos {
        FRONT_IDLE,
        FRONT,
        PIVOT_POS,
        BACK,
        BACK_IDLE
    }

    private passThruState currentPassThruState = passThruState.FRONT;
    private passThruPos currentPassThruPos = passThruPos.FRONT_IDLE;

    private void setPassThruAngle(double angle) {
        passThruServoR.turnToAngle(angle);
        passThruServoL.turnToAngle(angle);
    }

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


    public void runPassThruStates () {
        if (passThruInFront) {
            switch (currentPassThruState) {
                default:
                case BACK:
                case FRONT:
                    passThruTimer.reset();
                    break;
                case START:
                    passThruIsMoving = true;
                    passThruTimer.reset();
                    currentPassThruPos = passThruPos.PIVOT_POS;
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
                        passThruIsMoving = false;
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
                    passThruTimer.reset();
                    break;
                case START:
                    passThruIsMoving = true;
                    passThruTimer.reset();
                    currentPassThruPos = passThruPos.PIVOT_POS;
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
                        passThruIsMoving = false;
                        passThruTimer.reset();
                        currentPassThruState = passThruState.FRONT;
                    }
                    break;
            }
        }
        passThruSwitched = false;
    }

    public enum liftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

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

    public void setTargetLiftPos (double height) {
        targetLiftPos = height;
        targetLiftPosName = Double.toString(height);
        updateLiftProfile();
    }

    public void setLiftStateToCurrent() {
        liftState = new MotionState(currentLiftPos, currentLiftVelo, currentLiftAccel, currentLiftJerk);
    }

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

    public double getCurrentLiftPos () {
        return currentLiftPos;
    }

    public void readLiftPos () {
        double
                lastLiftPos = currentLiftPos,
                lastLiftVelo = currentLiftVelo,
                lastLiftAccel = currentLiftAccel,
                currentTimeStamp = liftDerivTimer.seconds(),
                dt = currentTimeStamp == lastTimestamp? 0.002: currentTimeStamp - lastTimestamp;
        lastTimestamp = currentTimeStamp;

        veloFilter.setGains(RobotConfig.LIFT_VELO_FILTER_GAIN, RobotConfig.LIFT_VELO_ESTIMATE_COUNT);
        accelFilter.setGains(RobotConfig.LIFT_ACCEL_FILTER_GAIN, RobotConfig.LIFT_ACCEL_ESTIMATE_COUNT);
        jerkFilter.setGains(RobotConfig.LIFT_JERK_FILTER_GAIN, RobotConfig.LIFT_JERK_ESTIMATE_COUNT);

        currentLiftPos = lift_motor2.encoder.getPosition() * RobotConfig.LIFT_TICKS_PER_INCH;
        currentLiftVelo = veloFilter.getEstimate((currentLiftPos - lastLiftPos) / dt);
        currentLiftAccel = accelFilter.getEstimate((currentLiftVelo - lastLiftVelo) / dt);
        currentLiftJerk = jerkFilter.getEstimate((currentLiftAccel - lastLiftAccel) / dt);
    }

    public void resetLift () {
        jerkFilter.resetPastValues();
        accelFilter.resetPastValues();
        veloFilter.resetPastValues();

        currentLiftJerk = 0.0;
        currentLiftAccel = 0.0;
        currentLiftVelo = 0.0;
        currentLiftPos = 0.0;

        lastTimestamp = 0.0;
        lift_motor2.resetEncoder();
        liftController.reset();

        setTargetLiftPos(liftPos.FLOOR);
    }

    public void runLiftToPos () {
        liftState = liftProfile.get(liftProfileTimer.seconds());

        liftController.setTargetPosition(liftState.getX());
        liftController.setTargetVelocity(liftState.getV());
        liftController.setTargetAcceleration(liftState.getA());

        if (liftController.atTargetPosition(currentLiftPos)) liftController.reset();

        updateLiftGains();
        runLift(liftController.update(currentLiftPos));
    }

    public void runLift (double veloCommand) {
        veloCommand += getLiftGravityFF();
        lift_motor1.set(veloCommand);
        lift_motor2.set(veloCommand);
        lift_motor3.set(veloCommand);
    }

    private double getLiftGravityFF () {
        double veloCommand = 0.0;

        if      (currentLiftPos >= RobotConfig.STAGES_FOUR)         veloCommand = RobotConfig.LIFT_kG_FOUR;
        else if (currentLiftPos >= RobotConfig.STAGES_THREE)        veloCommand = RobotConfig.LIFT_kG_THREE;
        else if (currentLiftPos >= RobotConfig.STAGES_TWO)          veloCommand = RobotConfig.LIFT_kG_TWO;
        else if (currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE)   veloCommand = RobotConfig.LIFT_kG_ONE;

        return veloCommand;
    }

    public void toggleTilt () {
        clawIsTilted ^= true;
    }

    public void toggleClaw () {
        clawIsOpen ^= true;
    }

    public void triggerClaw () {
        if (clawIsOpen) liftClaw(); else dropClaw();
    }

    public void setClawOpen (boolean open) {
        clawIsOpen = open;
    }

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
            liftClawTimer.reset();
            clawHasLifted = true;
        }
    }

    public void raiseClaw () {
        setTargetLiftPos(Math.min(
                currentLiftPos + ((currentLiftPos > RobotConfig.LIFT_POS_TOLERANCE)? 6: 2),
                RobotConfig.HEIGHT_TALL
        ));
    }

    public void liftClaw () {
        setClawOpen(false);
        liftClawTimer.reset();
    }

    public void dropClaw () {
        dropClaw(liftPos.FLOOR);
    }

    public void dropClaw (liftPos height) {
        setClawOpen(true);
        setTargetLiftPos(height);
    }

    public void togglePivot () {
        pivotIsFront ^= true;
    }

    public void runPivot () {
        pivotServo.turnToAngle(pivotIsFront? RobotConfig.PIVOT_FRONT_ANGLE: RobotConfig.PIVOT_BACK_ANGLE);
    }

    public void triggerPassThru () {
        if ((currentPassThruState != passThruState.FRONT) && (currentPassThruState != passThruState.BACK)) {
            passThruInFront ^= true;
            passThruSwitched = true;
        }
        else currentPassThruState = passThruState.START;
    }

    public void togglePassThru () {
        if (currentPassThruState != passThruState.FRONT) {
            currentPassThruPos = passThruPos.FRONT;
            currentPassThruState = passThruState.FRONT;
        } else {
            currentPassThruPos = passThruPos.BACK;
            currentPassThruState = passThruState.BACK;
        }
    }

    public void runConeArms (boolean down) {
        coneArmL.turnToAngle(down? RobotConfig.ARM_L_DOWN_ANGLE: RobotConfig.ARM_L_UP_ANGLE);
        coneArmR.turnToAngle(down? RobotConfig.ARM_R_DOWN_ANGLE: RobotConfig.ARM_R_UP_ANGLE);
    }

    public void printTelemetry (MultipleTelemetry telemetry) {
        telemetry.addData("Lift current position (in)", currentLiftPos);
        telemetry.addData("Lift profile position (in)", liftState.getX());
        telemetry.addData("Lift target position (name)", targetLiftPosName);
        telemetry.addLine();
        telemetry.addData("Lift position error (in)", liftController.getCurrentFilterEstimate());
        telemetry.addLine();
        telemetry.addData("Lift current velocity (in/s)", currentLiftVelo);
        telemetry.addData("Lift profile velocity (in/s)", liftState.getV());
        telemetry.addLine();
        telemetry.addData("Lift current acceleration (in/s^2)", currentLiftAccel);
        telemetry.addLine();
        telemetry.addData("Lift current jerk (in/s^3)", currentLiftJerk);
    }
}
