package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;

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

import org.firstinspires.ftc.teamcode.control.PIDFController;

/**
 * Contains 3-motor automated lift, 3-state claw, and state-machine-controlled passthrough functions
 * @author Arshad Anas
 * @since 2022/12/24
 */

public class PowerplayScorer {

    private MotorEx lift_motor1;
    private MotorEx lift_motor2;
    private MotorEx lift_motor3;
    private SimpleServo clawServo;
    private SimpleServo pivotServo;
    private SimpleServo passThruServoR;
    private SimpleServo passThruServoL;
    private PIDFController liftController;
    private MotionProfile liftProfile;
    private MotionState liftState;
    private ElapsedTime liftProfileTimer;
    private ElapsedTime liftDerivTimer;
    public DigitalChannel limitSwitch;
    public DigitalChannel LED1red;
    public DigitalChannel LED1green;
    public DigitalChannel LED2red;
    public DigitalChannel LED2green;
    private double lastTimestamp;
    private double lastLiftJerk;
    private double currentLiftJerk;
    private double lastLiftAccel;
    private double currentLiftAccel;
    private double lastLiftVelo;
    private double currentLiftVelo;
    private double lastLiftPos;
    private double currentLiftPos;
    private double targetLiftPos;
    private String targetLiftPosName;
    private static ElapsedTime passThruTimer;
    private static ElapsedTime liftClawTimer;
    private static ElapsedTime dropClawTimer;
    public boolean clawIsOpen;
    public boolean passThruIsMoving;
    private boolean passThruInFront;
    private boolean pivotIsFront;
    private boolean skipCurrentPassThruState;
    public boolean useLiftPIDF;
    private boolean clawHasLifted;
    private boolean clawHasDropped;

    public void init (HardwareMap hw) {

        clawServo = new SimpleServo(hw,"claw right",0,300);
        pivotServo = new SimpleServo(hw, "claw pivot",0,300);
        passThruServoR = new SimpleServo(hw, "passthrough 1",0,300);
        passThruServoL = new SimpleServo(hw, "passthrough 2",0,300);

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
                        TeleOpConfig.LIFT_kP,
                        TeleOpConfig.LIFT_kI,
                        TeleOpConfig.LIFT_kD
                ),
                TeleOpConfig.LIFT_INTEGRATION_MAX_VELO,
                TeleOpConfig.LIFT_PID_FILTER_GAIN,
                TeleOpConfig.LIFT_kV,
                TeleOpConfig.LIFT_kA,
                TeleOpConfig.LIFT_kS
        );
        liftController.setPositionTolerance(TeleOpConfig.LIFT_POS_TOLERANCE);
        liftController.setOutputBounds(-1.0, 1.0);

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
        clawHasDropped = true;
        useLiftPIDF = true;
        skipCurrentPassThruState = false;
        pivotIsFront = true;
        passThruInFront = true;
        passThruIsMoving = false;
        clawIsOpen = true;

        passThruTimer = new ElapsedTime();
        passThruTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();
        dropClawTimer = new ElapsedTime();
        dropClawTimer.reset();
        liftProfileTimer = new ElapsedTime();
        liftProfileTimer.reset();
        liftDerivTimer = new ElapsedTime();
        liftDerivTimer.reset();

        resetLift();
    }

    //  lift motor encoder resolution (ticks):
    private static final double LIFT_TICKS = 145.1;

    private enum passThruState {
        MOVING_TO_FRONT,
        IN_FRONT,
        MOVING_TO_PIVOT,
        PIVOTING,
        MOVING_TO_BACK,
        IN_BACK
    }
    private enum passThruPos {
        FRONT,
        PIVOT_POS,
        BACK
    }

    private passThruState currentPassThruState = passThruState.IN_FRONT;
    private passThruPos currentPassThruPos = passThruPos.FRONT;

    public void runPassThruServos () {
        switch (currentPassThruPos) {
            case FRONT:
                passThruServoR.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                passThruServoL.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                break;
            case PIVOT_POS:
                if (currentLiftPos >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                    passThruServoR.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                    passThruServoL.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                } else {
                    passThruServoR.turnToAngle(TeleOpConfig.PASS_RIGHT_PIVOT_ANGLE);
                    passThruServoL.turnToAngle(TeleOpConfig.PASS_LEFT_PIVOT_ANGLE);
                }
                break;
            case BACK:
                passThruServoR.turnToAngle(TeleOpConfig.PASS_RIGHT_BACK_ANGLE);
                passThruServoL.turnToAngle(TeleOpConfig.PASS_LEFT_BACK_ANGLE);
                break;
            default:
                currentPassThruPos = passThruPos.FRONT;
                break;
        }
    }


    public void runPassThruStates () {
        if (passThruInFront) {
            switch (currentPassThruState) {
                case IN_FRONT:
                    passThruTimer.reset();
                    skipCurrentPassThruState = false;
                    pivotIsFront = true;
                    passThruInFront = true;
                    break;
                case IN_BACK:
                    passThruTimer.reset();
                    skipCurrentPassThruState = false;
                    pivotIsFront = false;
                    passThruInFront = false;
                    break;
                case MOVING_TO_FRONT:
                    passThruTimer.reset();
                    currentPassThruState = passThruState.MOVING_TO_PIVOT;
                    currentPassThruPos = passThruPos.PIVOT_POS;
                    skipCurrentPassThruState = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) || skipCurrentPassThruState || currentLiftPos >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passThruTimer.reset();
                        currentPassThruState = passThruState.PIVOTING;
                        pivotIsFront = false;
                        skipCurrentPassThruState = false;
                    }
                    break;
                case PIVOTING:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_BACK_TIME) || skipCurrentPassThruState) {
                        passThruTimer.reset();
                        currentPassThruState = passThruState.MOVING_TO_BACK;
                        currentPassThruPos = passThruPos.BACK;
                        pivotIsFront = false;
                        skipCurrentPassThruState = false;
                    }
                    break;
                case MOVING_TO_BACK:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_BACK_TIME) || skipCurrentPassThruState) {
                        passThruTimer.reset();
                        passThruIsMoving = false;
                        currentPassThruState = passThruState.IN_BACK;
                        passThruInFront = false;
                        pivotIsFront = false;
                        skipCurrentPassThruState = false;
                    }
                    break;
                default:
                    currentPassThruState = passThruState.IN_FRONT;
                    break;
            }
        } else {
            switch (currentPassThruState) {
                case IN_BACK:
                    passThruTimer.reset();
                    skipCurrentPassThruState = false;
                    pivotIsFront = false;
                    passThruInFront = false;
                    break;
                case IN_FRONT:
                    passThruTimer.reset();
                    skipCurrentPassThruState = false;
                    pivotIsFront = true;
                    passThruInFront = true;
                    break;
                case MOVING_TO_BACK:
                    passThruTimer.reset();
                    currentPassThruState = passThruState.MOVING_TO_PIVOT;
                    currentPassThruPos = passThruPos.PIVOT_POS;
                    skipCurrentPassThruState = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.BACK_TO_PIVOT_TIME) || skipCurrentPassThruState) {
                        passThruTimer.reset();
                        currentPassThruState = passThruState.PIVOTING;
                        pivotIsFront = true;
                        skipCurrentPassThruState = false;
                    }
                    break;
                case PIVOTING:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_FRONT_TIME) || skipCurrentPassThruState) {
                        passThruTimer.reset();
                        currentPassThruState = passThruState.MOVING_TO_FRONT;
                        currentPassThruPos = passThruPos.FRONT;
                        pivotIsFront = true;
                        skipCurrentPassThruState = false;
                    }
                    break;
                case MOVING_TO_FRONT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (currentLiftPos >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) || skipCurrentPassThruState) {
                        passThruTimer.reset();
                        passThruIsMoving = false;
                        currentPassThruState = passThruState.IN_FRONT;
                        passThruInFront = true;
                        pivotIsFront = true;
                        skipCurrentPassThruState = false;
                    }
                    break;
                default:
                    currentPassThruState = passThruState.IN_BACK;
                    break;
            }
        }
    }

    public enum liftPos {
        FLOOR, TWO, THREE, FOUR, FIVE, LOW, MED, TALL
    }

    public void setTargetLiftPos (liftPos height) {
        switch (height){
            case FLOOR:
                targetLiftPos = TeleOpConfig.HEIGHT_FLOOR;
                targetLiftPosName = liftPos.FLOOR.name();
                break;
            case TWO:
                targetLiftPos = TeleOpConfig.HEIGHT_TWO;
                targetLiftPosName = liftPos.TWO.name();
                break;
            case THREE:
                targetLiftPos = TeleOpConfig.HEIGHT_THREE;
                targetLiftPosName = liftPos.THREE.name();
                break;
            case FOUR:
                targetLiftPos = TeleOpConfig.HEIGHT_FOUR;
                targetLiftPosName = liftPos.FOUR.name();
                break;
            case FIVE:
                targetLiftPos = TeleOpConfig.HEIGHT_FIVE;
                targetLiftPosName = liftPos.FIVE.name();
                break;
            case LOW:
                targetLiftPos = TeleOpConfig.HEIGHT_LOW;
                targetLiftPosName = liftPos.LOW.name();
                break;
            case MED:
                targetLiftPos = TeleOpConfig.HEIGHT_MEDIUM;
                targetLiftPosName = liftPos.MED.name();
                break;
            case TALL:
                targetLiftPos = TeleOpConfig.HEIGHT_TALL;
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

    public void updateLiftProfile () {
        double maxV = TeleOpConfig.LIFT_MAX_UP_VELO;
        double maxA = TeleOpConfig.LIFT_MAX_UP_ACCEL;
        
        if (targetLiftPos == currentLiftPos) {
            targetLiftPos += 0.25;
        } else if (targetLiftPos < currentLiftPos) {
            double maxV = TeleOpConfig.LIFT_MAX_DOWN_VELO;
            double maxA = TeleOpConfig.LIFT_MAX_DOWN_ACCEL;
        }

        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(currentLiftPos, currentLiftVelo, currentLiftAccel, currentLiftJerk),
            new MotionState(targetLiftPos, 0, 0, 0),
            maxV,
            maxA,
            TeleOpConfig.LIFT_MAX_JERK
        );

        liftProfileTimer.reset();
    }

    public void updateLiftGains () {
        liftController.setGains(
                new PIDCoefficients(
                        TeleOpConfig.LIFT_kP,
                        TeleOpConfig.LIFT_kI,
                        TeleOpConfig.LIFT_kD
                ),
                TeleOpConfig.LIFT_INTEGRATION_MAX_VELO,
                TeleOpConfig.LIFT_PID_FILTER_GAIN,
                TeleOpConfig.LIFT_kV,
                TeleOpConfig.LIFT_kA,
                TeleOpConfig.LIFT_kS
        );
        liftController.setPositionTolerance(TeleOpConfig.LIFT_POS_TOLERANCE);
    }

    public double getTargetLiftPos () {
        return targetLiftPos;
    }

    public double getCurrentLiftPos () {
        return currentLiftPos;
    }

    public void readLiftPos () {
        currentLiftPos = lift_motor2.encoder.getPosition() * TeleOpConfig.LIFT_TICKS_PER_INCH;

        double currentTimeStamp = liftDerivTimer.seconds();
        double dt = currentTimeStamp - lastTimestamp;
        if (dt == 0) {
            dt = 0.002;
        }
        currentLiftVelo = (currentLiftPos - lastLiftPos) / dt;
        currentLiftVelo = (TeleOpConfig.LIFT_VELO_FILTER_GAIN * lastLiftVelo) + ((1-TeleOpConfig.LIFT_VELO_FILTER_GAIN) * currentLiftVelo);
        currentLiftAccel = (currentLiftVelo - lastLiftVelo) / dt;
        currentLiftAccel = (TeleOpConfig.LIFT_ACCEL_FILTER_GAIN * lastLiftAccel) + ((1-TeleOpConfig.LIFT_ACCEL_FILTER_GAIN) * currentLiftAccel);
        currentLiftJerk = (currentLiftAccel - lastLiftAccel) / dt;
        currentLiftJerk = (TeleOpConfig.LIFT_JERK_FILTER_GAIN * lastLiftJerk) + ((1-TeleOpConfig.LIFT_JERK_FILTER_GAIN) * currentLiftJerk);

        lastLiftPos = currentLiftPos;
        lastLiftVelo = currentLiftVelo;
        lastLiftAccel = currentLiftAccel;
        lastLiftJerk = currentLiftJerk;
        lastTimestamp = currentTimeStamp;
    }

    public void resetLift () {
        lift_motor2.resetEncoder();
        lastLiftJerk = 0.0;
        currentLiftJerk = 0.0;
        lastLiftAccel = 0.0;
        currentLiftAccel = 0.0;
        lastLiftVelo = 0.0;
        currentLiftVelo = 0.0;
        lastLiftPos = 0.0;
        currentLiftPos = 0.0;
        lastTimestamp = 0.0;
        liftController.reset();
        setTargetLiftPos(liftPos.FLOOR);
    }

    public void runLiftToPos () {
        liftState = liftProfile.get(liftProfileTimer.seconds());

        liftController.setTargetPosition(liftState.getX());
        liftController.setTargetVelocity(liftState.getV());
        liftController.setTargetAcceleration(liftState.getA());

        if (liftController.atTargetPosition(currentLiftPos)) {
            liftController.reset();
        }

        if (useLiftPIDF) {
            updateLiftGains();
            runLift(liftController.update(currentLiftPos));
        }
    }

    public void runLift (double veloCommand) {
        veloCommand += getLiftGravityFF();
        lift_motor1.set(veloCommand);
        lift_motor2.set(veloCommand);
        lift_motor3.set(veloCommand);
    }

    private double getLiftGravityFF () {
        double veloCommand = 0.0;

        if (currentLiftPos >= TeleOpConfig.STAGES_FOUR) {
            veloCommand = TeleOpConfig.LIFT_kG_FOUR;
        } else if (currentLiftPos >= TeleOpConfig.STAGES_THREE) {
            veloCommand = TeleOpConfig.LIFT_kG_THREE;
        } else if (currentLiftPos >= TeleOpConfig.STAGES_TWO) {
            veloCommand = TeleOpConfig.LIFT_kG_TWO;
        } else if (currentLiftPos > TeleOpConfig.LIFT_POS_TOLERANCE) {
            veloCommand = TeleOpConfig.LIFT_kG_ONE;
        }

        return veloCommand;
    }

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    public void runClaw () {
        if (!clawIsOpen){
            clawServo.turnToAngle(TeleOpConfig.CLAW_CLOSED_ANGLE);
        } else if (passThruIsMoving) {
            clawServo.turnToAngle(TeleOpConfig.CLAW_PASS_ANGLE);
        } else {
            clawServo.turnToAngle(TeleOpConfig.CLAW_OPEN_ANGLE);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !clawHasLifted) {
            double heightIncrease = 2;

            if (currentLiftPos > TeleOpConfig.LIFT_POS_TOLERANCE) {
                heightIncrease = 6;
            }

            setTargetLiftPos(Math.min(getTargetLiftPos() + heightIncrease, TeleOpConfig.HEIGHT_TALL));
            liftClawTimer.reset();
            clawHasLifted = true;
        }

        if ((dropClawTimer.seconds() >= TeleOpConfig.CLAW_DROP_TIME) && !clawHasDropped) {
            clawIsOpen = true;
            clawHasDropped = true;
        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        clawHasLifted = false;
    }

    public void dropClaw () {
        setTargetLiftPos(liftPos.FLOOR);
        dropClawTimer.reset();
        clawHasDropped = false;
    }

    public void dropClaw (liftPos height) {
        setTargetLiftPos(height);
        dropClawTimer.reset();
        clawHasDropped = false;
    }

    public void togglePivot () {
        pivotIsFront = !pivotIsFront;
    }

    public void runPivot () {
        if (pivotIsFront) {
            pivotServo.turnToAngle(TeleOpConfig.PIVOT_FRONT_ANGLE);
        } else {
            pivotServo.turnToAngle(TeleOpConfig.PIVOT_BACK_ANGLE);
        }
    }

    public void triggerPassThru () {
        if ((currentPassThruState != passThruState.IN_FRONT) && (currentPassThruState != passThruState.IN_BACK)) {
            passThruInFront = !passThruInFront;
            skipCurrentPassThruState = true;
        } else {
            passThruIsMoving = true;
            passThruTimer.reset();
            currentPassThruState = passThruState.MOVING_TO_PIVOT;
            currentPassThruPos = passThruPos.PIVOT_POS;
            skipCurrentPassThruState = false;
        }
    }

    public void togglePassThru () {
        if (currentPassThruState != passThruState.IN_FRONT) {
            currentPassThruPos = passThruPos.FRONT;
            currentPassThruState = passThruState.IN_FRONT;
        } else {
            currentPassThruPos = passThruPos.BACK;
            currentPassThruState = passThruState.IN_BACK;
        }
    }

    public void printTelemetry (MultipleTelemetry myTelemetry) {
        if (limitSwitch.getState()) {
            myTelemetry.addData("Limit switch", "is not triggered");
        } else {
            myTelemetry.addData("Limit switch", "is triggered");
        }
        myTelemetry.addLine();
        if (!clawIsOpen){
            myTelemetry.addData("Claw is", "closed");
        } else if (passThruIsMoving) {
            myTelemetry.addData("Claw is", "half-closed");
        } else {
            myTelemetry.addData("Claw is", "open");
        }
        myTelemetry.addLine();
        myTelemetry.addData("Lift current position (in)", currentLiftPos);
        myTelemetry.addData("Lift profile position (in)", liftState.getX());
        myTelemetry.addData("Lift target position (name)", targetLiftPosName);
        myTelemetry.addLine();
        myTelemetry.addData("Lift position error (in)", liftController.getCurrentFilterEstimate());
        myTelemetry.addLine();
        myTelemetry.addData("Lift current velocity (in/s)", currentLiftVelo);
        myTelemetry.addData("Lift profile velocity (in/s)", liftState.getV());
        myTelemetry.addLine();
        myTelemetry.addData("Lift current acceleration (in/s^2)", currentLiftAccel);
        myTelemetry.addLine();
        myTelemetry.addData("Lift current jerk (in/s^3)", currentLiftJerk);
        myTelemetry.addLine();
        myTelemetry.addData("Passthrough status", currentPassThruState);
    }
}
