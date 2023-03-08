package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Creates a 'PowerplayScorer' class that contains lift, claw, and passthrough functions
 * @author Arshad Anas
 * @since 2022/12/24
 */

public class PowerplayScorer {

    private MotorEx lift_motor1;
    private MotorEx lift_motor2;
    private MotorEx lift_motor3;
    private SimpleServo clawRight;
    private SimpleServo clawPivot;
    private SimpleServo passthroughRight;
    private SimpleServo passthroughLeft;
    private PIDFController liftController;
    private String targetLiftPositionName;
    public DigitalChannel limitSwitch;
    public DigitalChannel LED1red;
    public DigitalChannel LED1green;
    public DigitalChannel LED2red;
    public DigitalChannel LED2green;
    private double currentLiftPosition = 0;
    private double targetLiftPosition;
    public double liftVelocity;
    private static ElapsedTime passthroughTimer;
    private static ElapsedTime liftClawTimer;
    public boolean clawIsOpen = true;
    public boolean clawIsFlipping = false;
    private boolean passthroughInFront = true;
    private boolean pivotIsFront = true;
    // override variable--when true, skips the timer to switch to next state immediately
    private boolean skipPassthroughState = false;
    public boolean useLiftPIDF = true;

    // the following is the code that runs during initialization
    public void init(HardwareMap hw) {

        clawRight = new SimpleServo(hw,"claw right",0,300);
        clawPivot = new SimpleServo(hw, "claw pivot",0,300);
        passthroughRight = new SimpleServo(hw, "passthrough 1",0,300);
        passthroughLeft = new SimpleServo(hw, "passthrough 2",0,300);

        lift_motor1 = new MotorEx(hw, "lift motor 1", LIFT_TICKS, MAX_RPM);
        lift_motor2 = new MotorEx(hw, "lift motor 2", LIFT_TICKS, MAX_RPM);
        lift_motor3 = new MotorEx(hw, "lift motor 3", LIFT_TICKS, MAX_RPM);

        liftController = new PIDFController(
                TeleOpConfig.LIFT_P,
                TeleOpConfig.LIFT_I,
                TeleOpConfig.LIFT_D,
                TeleOpConfig.LIFT_F
        );
        liftController.setPIDF(
                TeleOpConfig.LIFT_P,
                TeleOpConfig.LIFT_I,
                TeleOpConfig.LIFT_D,
                TeleOpConfig.LIFT_F
        );
        liftController.setTolerance(TeleOpConfig.LIFT_E_TOLERANCE, TeleOpConfig.LIFT_V_TOLERANCE);

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
        targetLiftPosition = TeleOpConfig.HEIGHT_ONE;
        targetLiftPositionName = liftPositions.ONE.name();

        passthroughTimer = new ElapsedTime();
        passthroughTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();

        limitSwitch = hw.get(DigitalChannel.class, "limit switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        LED1red = hw.get(DigitalChannel.class, "red1");
        LED1green = hw.get(DigitalChannel.class, "green1");
        LED1red.setMode(DigitalChannel.Mode.OUTPUT);
        LED1green.setMode(DigitalChannel.Mode.OUTPUT);

        LED2red = hw.get(DigitalChannel.class, "red2");
        LED2green = hw.get(DigitalChannel.class, "green2");
        LED2red.setMode(DigitalChannel.Mode.OUTPUT);
        LED2green.setMode(DigitalChannel.Mode.OUTPUT);

    }

    //  lift motor encoder resolution (ticks):
    private static final double LIFT_TICKS = 145.1;



    private enum passthroughStates {
        MOVING_TO_FRONT,
        IN_FRONT,
        MOVING_TO_PIVOT,
        PIVOTING,
        MOVING_TO_BACK,
        IN_BACK
    }
    private enum passthroughPositions {
        FRONT,
        PIVOT_POS,
        BACK
    }

    private passthroughStates currentPassthroughState = passthroughStates.IN_FRONT;
    private passthroughPositions currentPassthroughPosition = passthroughPositions.FRONT;

    public passthroughStates getCurrentPassthroughState () {
        return currentPassthroughState;
    }

    public void runPassthroughServos() {
        switch (currentPassthroughPosition) {
            case FRONT:
                passthroughRight.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                passthroughLeft.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                break;
            case PIVOT_POS:
                if (currentLiftPosition >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                    passthroughRight.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                    passthroughLeft.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                } else {
                    passthroughRight.turnToAngle(TeleOpConfig.PASS_RIGHT_PIVOT_ANGLE);
                    passthroughLeft.turnToAngle(TeleOpConfig.PASS_LEFT_PIVOT_ANGLE);
                }
                break;
            case BACK:
                passthroughRight.turnToAngle(TeleOpConfig.PASS_RIGHT_BACK_ANGLE);
                passthroughLeft.turnToAngle(TeleOpConfig.PASS_LEFT_BACK_ANGLE);
                break;
            default:
                currentPassthroughPosition = passthroughPositions.FRONT;
                break;
        }
    }


    public void runPassthroughStates() {
        if (passthroughInFront) {
            switch (currentPassthroughState) {
                case IN_FRONT:
                    passthroughTimer.reset();
                    skipPassthroughState = false;
                    pivotIsFront = true;
                    passthroughInFront = true;
                    break;
                case IN_BACK:
                    passthroughTimer.reset();
                    skipPassthroughState = false;
                    pivotIsFront = false;
                    passthroughInFront = false;
                    break;
                case MOVING_TO_FRONT:
                    passthroughTimer.reset();
                    currentPassthroughState = passthroughStates.MOVING_TO_PIVOT;
                    currentPassthroughPosition = passthroughPositions.PIVOT_POS;
                    skipPassthroughState = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passthroughTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) || skipPassthroughState || currentLiftPosition >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passthroughTimer.reset();
                        currentPassthroughState = passthroughStates.PIVOTING;
                        pivotIsFront = false;
                        skipPassthroughState = false;
                    }
                    break;
                case PIVOTING:
                    if ((passthroughTimer.seconds() >= TeleOpConfig.PIVOTING_TO_BACK_TIME) || skipPassthroughState) {
                        passthroughTimer.reset();
                        currentPassthroughState = passthroughStates.MOVING_TO_BACK;
                        currentPassthroughPosition = passthroughPositions.BACK;
                        pivotIsFront = false;
                        skipPassthroughState = false;
                    }
                    break;
                case MOVING_TO_BACK:
                    if ((passthroughTimer.seconds() >= TeleOpConfig.PIVOT_TO_BACK_TIME) || skipPassthroughState) {
                        passthroughTimer.reset();
                        clawIsFlipping = false;
                        currentPassthroughState = passthroughStates.IN_BACK;
                        passthroughInFront = false;
                        pivotIsFront = false;
                        skipPassthroughState = false;
                    }
                    break;
                default:
                    currentPassthroughState = passthroughStates.IN_FRONT;
                    break;
            }
        } else {
            switch (currentPassthroughState) {
                case IN_BACK:
                    passthroughTimer.reset();
                    skipPassthroughState = false;
                    pivotIsFront = false;
                    passthroughInFront = false;
                    break;
                case IN_FRONT:
                    passthroughTimer.reset();
                    skipPassthroughState = false;
                    pivotIsFront = true;
                    passthroughInFront = true;
                    break;
                case MOVING_TO_BACK:
                    passthroughTimer.reset();
                    currentPassthroughState = passthroughStates.MOVING_TO_PIVOT;
                    currentPassthroughPosition = passthroughPositions.PIVOT_POS;
                    skipPassthroughState = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passthroughTimer.seconds() >= TeleOpConfig.BACK_TO_PIVOT_TIME) || skipPassthroughState) {
                        passthroughTimer.reset();
                        currentPassthroughState = passthroughStates.PIVOTING;
                        pivotIsFront = true;
                        skipPassthroughState = false;
                    }
                    break;
                case PIVOTING:
                    if ((passthroughTimer.seconds() >= TeleOpConfig.PIVOTING_TO_FRONT_TIME) || skipPassthroughState) {
                        passthroughTimer.reset();
                        currentPassthroughState = passthroughStates.MOVING_TO_FRONT;
                        currentPassthroughPosition = passthroughPositions.FRONT;
                        pivotIsFront = true;
                        skipPassthroughState = false;
                    }
                    break;
                case MOVING_TO_FRONT:
                    if ((passthroughTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (currentLiftPosition >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) || skipPassthroughState) {
                        passthroughTimer.reset();
                        clawIsFlipping = false;
                        currentPassthroughState = passthroughStates.IN_FRONT;
                        passthroughInFront = true;
                        pivotIsFront = true;
                        skipPassthroughState = false;
                    }
                    break;
                default:
                    currentPassthroughState = passthroughStates.IN_BACK;
                    break;
            }
        }
    }


    public enum liftPositions {
            ONE, TWO, THREE, FOUR, FIVE, GROUND, LOW, MED, TALL
    }

    public void setTargetLiftPosition(liftPositions height) {

        switch (height){
            case ONE:
                targetLiftPosition = TeleOpConfig.HEIGHT_ONE;
                targetLiftPositionName = liftPositions.ONE.name();
                break;
            case TWO:
                targetLiftPosition = TeleOpConfig.HEIGHT_TWO;
                targetLiftPositionName = liftPositions.TWO.name();
                break;
            case THREE:
                targetLiftPosition = TeleOpConfig.HEIGHT_THREE;
                targetLiftPositionName = liftPositions.THREE.name();
                break;
            case FOUR:
                targetLiftPosition = TeleOpConfig.HEIGHT_FOUR;
                targetLiftPositionName = liftPositions.FOUR.name();
                break;
            case FIVE:
                targetLiftPosition = TeleOpConfig.HEIGHT_FIVE;
                targetLiftPositionName = liftPositions.FIVE.name();
                break;
            case GROUND:
                targetLiftPosition = TeleOpConfig.HEIGHT_GROUND;
                targetLiftPositionName = liftPositions.GROUND.name();
                break;
            case LOW:
                targetLiftPosition = TeleOpConfig.HEIGHT_LOW;
                targetLiftPositionName = liftPositions.LOW.name();
                break;
            case MED:
                targetLiftPosition = TeleOpConfig.HEIGHT_MEDIUM;
                targetLiftPositionName = liftPositions.MED.name();
                break;
            case TALL:
                targetLiftPosition = TeleOpConfig.HEIGHT_TALL;
                targetLiftPositionName = liftPositions.TALL.name();
                break;
        }
    }

    public void setTargetLiftPosition(double height) {
        targetLiftPosition = height;
        targetLiftPositionName = Double.toString(height);
    }

    public double getTargetLiftPosition() {
        return targetLiftPosition;
    }

    public double getCurrentLiftPosition() {
        return currentLiftPosition;
    }

    public String getTargetLiftPositionName() {
        return targetLiftPositionName;
    }

    public void runLiftToPosition() {
        currentLiftPosition = lift_motor2.encoder.getPosition() * TeleOpConfig.LIFT_TICKS_PER_INCH;
        liftController.setSetPoint(targetLiftPosition);

        if (useLiftPIDF && !liftController.atSetPoint()) {
            liftVelocity = liftController.calculate(currentLiftPosition);

            if (liftVelocity < TeleOpConfig.LIFT_MAX_DOWN_VELOCITY) {
                liftVelocity = TeleOpConfig.LIFT_MAX_DOWN_VELOCITY;
            }

            runLift(liftVelocity);
        }
    }

    public void runLift(double velocity) {
        lift_motor1.set(velocity);
        lift_motor2.set(velocity);
        lift_motor3.set(velocity);
    }

    public void resetLiftEncoder() {
        lift_motor2.resetEncoder();
        currentLiftPosition = 0;
    }

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    private boolean clawHasLifted = true;

    public void runClaw () {
        if (!clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_CLOSED_ANGLE);
        } else if (clawIsFlipping) {
            clawRight.turnToAngle(TeleOpConfig.CLAW_PASS_ANGLE);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_OPEN_ANGLE);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !clawHasLifted) {
            setTargetLiftPosition(getTargetLiftPosition() + 5);
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
        targetLiftPosition = TeleOpConfig.HEIGHT_ONE;
        clawIsOpen = true;
    }


    public void togglePivot () {
        pivotIsFront = !pivotIsFront;
    }

    public void runPivot () {
        if(pivotIsFront) {
            clawPivot.turnToAngle(TeleOpConfig.PIVOT_FRONT_ANGLE);
        } else {
            clawPivot.turnToAngle(TeleOpConfig.PIVOT_BACK_ANGLE);
        }
    }

    public void triggerPassthrough() {
        if ((currentPassthroughState != passthroughStates.IN_FRONT) && (currentPassthroughState != passthroughStates.IN_BACK)) {
            passthroughInFront = !passthroughInFront;
            skipPassthroughState = true;
        } else {
            clawIsFlipping = true;
            passthroughTimer.reset();
            currentPassthroughState = passthroughStates.MOVING_TO_PIVOT;
            currentPassthroughPosition = passthroughPositions.PIVOT_POS;
            skipPassthroughState = false;
        }
    }

    public void togglePassthrough () {
        if (currentPassthroughState != passthroughStates.IN_FRONT) {
            currentPassthroughPosition = passthroughPositions.FRONT;
            currentPassthroughState = passthroughStates.IN_FRONT;
        } else {
            currentPassthroughPosition = passthroughPositions.BACK;
            currentPassthroughState = passthroughStates.IN_BACK;
        }
    }
}
