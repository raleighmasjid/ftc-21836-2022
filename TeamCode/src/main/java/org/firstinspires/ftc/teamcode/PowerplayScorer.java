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

    public MotorEx lift_motor1;
    public MotorEx lift_motor2;
    public MotorEx lift_motor3;
    public SimpleServo clawRight;
    public SimpleServo clawPivot;
    public SimpleServo passThruRight;
    public SimpleServo passThruLeft;
    public PIDFController liftController;
    public String targetLiftPosName;
    public DigitalChannel limitSwitch;
    public double targetLiftPos;
    public double liftVelocity;
    public static ElapsedTime passThruTimer;
    public static ElapsedTime liftClawTimer;
    public static ElapsedTime dropClawTimer;
    public static ElapsedTime resetLiftTimer;

    // the following is the code that runs during initialization
    public void init(HardwareMap hw) {

        clawRight = new SimpleServo(hw,"claw right",0,300);
        clawPivot = new SimpleServo(hw, "claw pivot",0,300);
        passThruRight = new SimpleServo(hw, "passthrough 1",0,300);
        passThruLeft = new SimpleServo(hw, "passthrough 2",0,300);

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
        targetLiftPos = TeleOpConfig.HEIGHT_ONE;
        targetLiftPosName = liftHeights.ONE.name();

        passThruTimer = new ElapsedTime();
        passThruTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();
        dropClawTimer = new ElapsedTime();
        dropClawTimer.reset();
        resetLiftTimer = new ElapsedTime();
        resetLiftTimer.reset();

        limitSwitch = hw.get(DigitalChannel.class, "limit switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    //  lift motor encoder resolution (ticks):
    public static final double LIFT_TICKS = 145.1;

    public boolean clawIsOpen = true;
    public boolean clawIsPass = false;
    public boolean passIsFront = true;
    public boolean pivotIsFront = true;
    // override variable--when true, skips the timer to switch to next state immediately
    public boolean skip = false;
    public boolean useLiftPIDF = true;

    public enum passStates {
        MOVING_TO_FRONT,
        IN_FRONT,
        MOVING_TO_PIVOT,
        PIVOTING,
        MOVING_TO_BACK,
        IN_BACK
    }
    public enum passPositions {
        FRONT,
        PIVOT_POS,
        UP,
        BACK
    }

    public passStates currentPassState = passStates.IN_FRONT;
    public passPositions currentPassPos = passPositions.FRONT;

    public void runPassServos () {
        switch (currentPassPos) {
            case FRONT:
                passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                break;
            case PIVOT_POS:
                if (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                    passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                } else {
                    passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_PIVOT_ANGLE);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_PIVOT_ANGLE);
                }
                break;
            case UP:
                passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_UP_ANGLE);
                passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_UP_ANGLE);
                break;
            case BACK:
                passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_BACK_ANGLE);
                passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_BACK_ANGLE);
                break;
            default:
                currentPassPos = passPositions.FRONT;
                break;
        }
    }


    public void runPassStates() {
        if (passIsFront) {
            switch (currentPassState) {
                case IN_FRONT:
                case IN_BACK:
                    passThruTimer.reset();
                    skip = false;
                    break;
                case MOVING_TO_FRONT:
                    passThruTimer.reset();
                    currentPassState = passStates.MOVING_TO_PIVOT;
                    currentPassPos = passPositions.PIVOT_POS;
                    skip = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) || skip) {
                        passThruTimer.reset();
                        currentPassState = passStates.PIVOTING;
                        pivotIsFront = false;
                        skip = false;
                    }
                    break;
                case PIVOTING:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_BACK_TIME) || skip) {
                        passThruTimer.reset();
                        currentPassState = passStates.MOVING_TO_BACK;
                        currentPassPos = passPositions.BACK;
                        skip = false;
                    }
                    break;
                case MOVING_TO_BACK:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_BACK_TIME) || skip) {
                        passThruTimer.reset();
                        clawIsPass = false;
                        currentPassState = passStates.IN_BACK;
                        passIsFront = false;
                        skip = false;
                    }
                    break;
                default:
                    currentPassState = passStates.IN_FRONT;
                    break;
            }
        } else {
            switch (currentPassState) {
                case IN_BACK:
                case IN_FRONT:
                    passThruTimer.reset();
                    skip = false;
                    break;
                case MOVING_TO_BACK:
                    passThruTimer.reset();
                    currentPassState = passStates.MOVING_TO_PIVOT;
                    currentPassPos = passPositions.PIVOT_POS;
                    skip = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.BACK_TO_PIVOT_TIME) || skip) {
                        passThruTimer.reset();
                        currentPassState = passStates.PIVOTING;
                        pivotIsFront = true;
                        skip = false;
                    }
                    break;
                case PIVOTING:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_FRONT_TIME) || skip) {
                        passThruTimer.reset();
                        currentPassState = passStates.MOVING_TO_FRONT;
                        currentPassPos = passPositions.FRONT;
                        skip = false;
                    }
                    break;
                case MOVING_TO_FRONT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) || skip) {
                        passThruTimer.reset();
                        clawIsPass = false;
                        currentPassState = passStates.IN_FRONT;
                        passIsFront = true;
                        skip = false;
                    }
                    break;
                default:
                    currentPassState = passStates.IN_BACK;
                    break;
            }
        }
    }


    public enum liftHeights {
            ONE, TWO, THREE, FOUR, FIVE, GROUND, LOW, MED, TALL
    }

    public void setLiftPos(liftHeights height) {

        switch (height){
            case ONE:
                targetLiftPos = TeleOpConfig.HEIGHT_ONE;
                targetLiftPosName = liftHeights.ONE.name();
                break;
            case TWO:
                targetLiftPos = TeleOpConfig.HEIGHT_TWO;
                targetLiftPosName = liftHeights.TWO.name();
                break;
            case THREE:
                targetLiftPos = TeleOpConfig.HEIGHT_THREE;
                targetLiftPosName = liftHeights.THREE.name();
                break;
            case FOUR:
                targetLiftPos = TeleOpConfig.HEIGHT_FOUR;
                targetLiftPosName = liftHeights.FOUR.name();
                break;
            case FIVE:
                targetLiftPos = TeleOpConfig.HEIGHT_FIVE;
                targetLiftPosName = liftHeights.FIVE.name();
                break;
            case GROUND:
                targetLiftPos = TeleOpConfig.HEIGHT_GROUND;
                targetLiftPosName = liftHeights.GROUND.name();
                break;
            case LOW:
                targetLiftPos = TeleOpConfig.HEIGHT_LOW;
                targetLiftPosName = liftHeights.LOW.name();
                break;
            case MED:
                targetLiftPos = TeleOpConfig.HEIGHT_MEDIUM;
                targetLiftPosName = liftHeights.MED.name();
                break;
            case TALL:
                targetLiftPos = TeleOpConfig.HEIGHT_TALL;
                targetLiftPosName = liftHeights.TALL.name();
                break;
        }
    }

    boolean liftedPass = false;
    passPositions lastPassPos = currentPassPos;

    public void runLiftToPos() {
        liftController.setSetPoint(targetLiftPos);

        if (useLiftPIDF && !liftController.atSetPoint()) {
            liftVelocity = liftController.calculate(lift_motor2.getCurrentPosition());

            if (liftVelocity < TeleOpConfig.LIFT_MAX_DOWN_VELOCITY) {
                liftVelocity = TeleOpConfig.LIFT_MAX_DOWN_VELOCITY;
            }

            if (liftVelocity < 0 && !liftedPass && dropClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) {
                lastPassPos = currentPassPos;
                currentPassPos = passPositions.UP;
                liftedPass = true;
                clawIsPass = true;
            }
            if (liftedPass && liftVelocity >= 0) {
                currentPassPos = lastPassPos;
                liftedPass = false;
                clawIsPass = false;
            }

            runLift(liftVelocity);
        }
    }

    public void runLift(double velocity) {
        lift_motor1.set(velocity);
        lift_motor2.set(velocity);
        lift_motor3.set(velocity);
    }

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    public boolean hasLifted = true;
    public boolean hasDropped = true;

    public void runClaw () {
        if (!clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_CLOSED_ANGLE);
        } else if (clawIsPass) {
            clawRight.turnToAngle(TeleOpConfig.CLAW_PASS_ANGLE);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_OPEN_ANGLE);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !hasLifted) {
            targetLiftPos = liftController.getSetPoint() + 150;
            liftClawTimer.reset();
            hasLifted = true;
        }
//        if ((dropClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !hasDropped) {
//            currentPassPos = passPositions.UP;
//            dropClawTimer.reset();
//            hasDropped = true;
//        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        hasLifted = false;
    }

    public void dropClaw () {
        if (!clawIsOpen) {
            dropClawTimer.reset();
        }
        clawIsOpen = true;
        hasDropped = false;
        targetLiftPos = TeleOpConfig.HEIGHT_ONE;
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

    public void togglePassthrough () {
        if ((currentPassState != passStates.IN_FRONT) && (currentPassState != passStates.IN_BACK)) {
            passIsFront = !passIsFront;
            skip = true;
        } else {
            clawIsPass = true;
            passThruTimer.reset();
            currentPassState = passStates.MOVING_TO_PIVOT;
            currentPassPos = passPositions.PIVOT_POS;
            skip = false;
        }
    }
}
