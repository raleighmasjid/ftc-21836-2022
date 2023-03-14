package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;

import org.firstinspires.ftc.teamcode.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public DigitalChannel limitSwitch;
    public DigitalChannel LED1red;
    public DigitalChannel LED1green;
    public DigitalChannel LED2red;
    public DigitalChannel LED2green;
    private double currentLiftPos = 0;
    private double targetLiftPos;
    private String targetLiftPosName;
    private double liftVelocity;
    private static ElapsedTime passThruTimer;
    private static ElapsedTime liftClawTimer;
    public boolean clawIsOpen = true;
    public boolean passThruIsMoving = false;
    private boolean passThruInFront = true;
    private boolean pivotIsFront = true;
    private boolean skipCurrentPassThruState = false;
    public boolean useLiftPIDF = true;

    public void init(HardwareMap hw) {

        clawServo = new SimpleServo(hw,"claw right",0,300);
        pivotServo = new SimpleServo(hw, "claw pivot",0,300);
        passThruServoR = new SimpleServo(hw, "passthrough 1",0,300);
        passThruServoL = new SimpleServo(hw, "passthrough 2",0,300);

        lift_motor1 = new MotorEx(hw, "lift motor 1", LIFT_TICKS, MAX_RPM);
        lift_motor2 = new MotorEx(hw, "lift motor 2", LIFT_TICKS, MAX_RPM);
        lift_motor3 = new MotorEx(hw, "lift motor 3", LIFT_TICKS, MAX_RPM);

        PIDCoefficients liftCoefficients = new PIDCoefficients(
                TeleOpConfig.LIFT_kP,
                TeleOpConfig.LIFT_kI,
                TeleOpConfig.LIFT_kD
        );
        liftController = new PIDFController(liftCoefficients);

        lift_motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        lift_motor1.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor2.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor3.setRunMode(Motor.RunMode.VelocityControl);

        lift_motor1.setInverted(true);
        lift_motor2.setInverted(false);
        lift_motor3.setInverted(true);

        lift_motor2.resetEncoder();
        targetLiftPos = TeleOpConfig.HEIGHT_ONE;
        targetLiftPosName = liftPos.ONE.name();

        passThruTimer = new ElapsedTime();
        passThruTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();

        limitSwitch = hw.get(DigitalChannel.class, "limit switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        LED1red = hw.get(DigitalChannel.class, "LED1red");
        LED1green = hw.get(DigitalChannel.class, "LED1green");
        LED1red.setMode(DigitalChannel.Mode.OUTPUT);
        LED1green.setMode(DigitalChannel.Mode.OUTPUT);

        LED2red = hw.get(DigitalChannel.class, "LED2red");
        LED2green = hw.get(DigitalChannel.class, "LED2green");
        LED2red.setMode(DigitalChannel.Mode.OUTPUT);
        LED2green.setMode(DigitalChannel.Mode.OUTPUT);

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

    public passThruState getCurrentPassThruState() {
        return currentPassThruState;
    }

    public void runPassThruServos() {
        switch (currentPassThruPos) {
            case FRONT:
                passThruServoR.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                passThruServoL.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                break;
            case PIVOT_POS:
                if (getCurrentLiftPos() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
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


    public void runPassThruStates() {
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
                    if ((passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) || skipCurrentPassThruState || getCurrentLiftPos() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
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
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (getCurrentLiftPos() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) || skipCurrentPassThruState) {
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
            ONE, TWO, THREE, FOUR, FIVE, GROUND, LOW, MED, TALL
    }

    public void setTargetLiftPos(liftPos height) {

        switch (height){
            case ONE:
                targetLiftPos = TeleOpConfig.HEIGHT_ONE;
                targetLiftPosName = liftPos.ONE.name();
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
            case GROUND:
                targetLiftPos = TeleOpConfig.HEIGHT_GROUND;
                targetLiftPosName = liftPos.GROUND.name();
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
        
    }

    public void setTargetLiftPos(double height) {
        targetLiftPos = height;
        targetLiftPosName = Double.toString(height);
    }

    public double getTargetLiftPos() {
        return targetLiftPos;
    }

    public String getTargetLiftPosName() {
        return targetLiftPosName;
    }

    public double getCurrentLiftPos() {
        readLiftPos();
        return currentLiftPos;
    }

    private void readLiftPos () {
        currentLiftPos = lift_motor2.encoder.getPosition() * TeleOpConfig.LIFT_TICKS_PER_INCH;
    }

    public void resetLiftEncoder () {
        lift_motor2.resetEncoder();
        currentLiftPos = 0;
    }

    public boolean liftAtTargetPos() {
        return Math.abs(getTargetLiftPos() - getCurrentLiftPos()) <= TeleOpConfig.LIFT_E_TOLERANCE;
    }

    public void runLiftToPos() {
        readLiftPos();
        liftController.setTargetPosition(targetLiftPos);

        if (useLiftPIDF) {
            liftVelocity = 0;
            if (!liftAtTargetPos()) {
                liftVelocity = liftController.update(getCurrentLiftPos());

                if (liftVelocity < TeleOpConfig.LIFT_MAX_DOWN_VELOCITY) {
                    liftVelocity = TeleOpConfig.LIFT_MAX_DOWN_VELOCITY;
                }
            }

            liftVelocity += TeleOpConfig.LIFT_kG;
            runLift(liftVelocity);
        }
    }

    public void runLift (double velocity) {
        lift_motor1.set(velocity);
        lift_motor2.set(velocity);
        lift_motor3.set(velocity);
    }

    public double getLiftVelocity () {
        return liftVelocity;
    }

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    private boolean clawHasLifted = true;

    public void runClaw () {
        if (!clawIsOpen){
            clawServo.turnToAngle(TeleOpConfig.CLAW_CLOSED_ANGLE);
        } else if (passThruIsMoving) {
            clawServo.turnToAngle(TeleOpConfig.CLAW_PASS_ANGLE);
        } else {
            clawServo.turnToAngle(TeleOpConfig.CLAW_OPEN_ANGLE);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !clawHasLifted) {
            setTargetLiftPos(getTargetLiftPos() + 5);
            liftClawTimer.reset();
            clawHasLifted = true;
        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        clawHasLifted = false;
    }

    public void dropClaw () {
        setTargetLiftPos(liftPos.ONE);
        clawIsOpen = true;
    }

    public void dropClaw (liftPos height) {
        setTargetLiftPos(height);
        clawIsOpen = true;
    }

    public void togglePivot () {
        pivotIsFront = !pivotIsFront;
    }

    public void runPivot () {
        if(pivotIsFront) {
            pivotServo.turnToAngle(TeleOpConfig.PIVOT_FRONT_ANGLE);
        } else {
            pivotServo.turnToAngle(TeleOpConfig.PIVOT_BACK_ANGLE);
        }
    }

    public void triggerPassThru() {
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

    public void togglePassThru() {
        if (currentPassThruState != passThruState.IN_FRONT) {
            currentPassThruPos = passThruPos.FRONT;
            currentPassThruState = passThruState.IN_FRONT;
        } else {
            currentPassThruPos = passThruPos.BACK;
            currentPassThruState = passThruState.IN_BACK;
        }
    }
}
