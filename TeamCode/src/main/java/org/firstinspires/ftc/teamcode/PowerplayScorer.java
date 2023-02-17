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
    public boolean clawIsOpen = true;
    public boolean clawIsPass = false;
    public boolean pivotIsFront = true;
    public boolean passIsFront = true;
    public boolean passIsLifted = false;
    public boolean useLiftPIDF = true;
    public boolean skipCurrentPassState = false;
    public boolean clawHasLifted = true;
    public boolean clawHasDropped = true;
    public static ElapsedTime passThruTimer;
    public static ElapsedTime liftClawTimer;
    public static ElapsedTime dropClawTimer;
    public static ElapsedTime resetLiftTimer;
    public static final double LIFT_TICKS = 145.1;


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
    public passPositions lastPassPos = currentPassPos;

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
                if (passIsFront) {
                    passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_UP_ANGLE);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_UP_ANGLE);
                } else {
                    passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_BACK_UP_ANGLE);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_BACK_UP_ANGLE);
                }
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
                    skipCurrentPassState = false;
                    break;
                case MOVING_TO_FRONT:
                    passThruTimer.reset();
                    currentPassState = passStates.MOVING_TO_PIVOT;
                    currentPassPos = passPositions.PIVOT_POS;
                    skipCurrentPassState = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) || skipCurrentPassState) {
                        passThruTimer.reset();
                        currentPassState = passStates.PIVOTING;
                        pivotIsFront = false;
                        skipCurrentPassState = false;
                    }
                    break;
                case PIVOTING:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_BACK_TIME) || skipCurrentPassState) {
                        passThruTimer.reset();
                        currentPassState = passStates.MOVING_TO_BACK;
                        currentPassPos = passPositions.BACK;
                        skipCurrentPassState = false;
                    }
                    break;
                case MOVING_TO_BACK:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_BACK_TIME) || skipCurrentPassState) {
                        passThruTimer.reset();
                        clawIsPass = false;
                        currentPassState = passStates.IN_BACK;
                        passIsFront = false;
                        skipCurrentPassState = false;
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
                    skipCurrentPassState = false;
                    break;
                case MOVING_TO_BACK:
                    passThruTimer.reset();
                    currentPassState = passStates.MOVING_TO_PIVOT;
                    currentPassPos = passPositions.PIVOT_POS;
                    skipCurrentPassState = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.BACK_TO_PIVOT_TIME) || skipCurrentPassState) {
                        passThruTimer.reset();
                        currentPassState = passStates.PIVOTING;
                        pivotIsFront = true;
                        skipCurrentPassState = false;
                    }
                    break;
                case PIVOTING:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_FRONT_TIME) || skipCurrentPassState) {
                        passThruTimer.reset();
                        currentPassState = passStates.MOVING_TO_FRONT;
                        currentPassPos = passPositions.FRONT;
                        skipCurrentPassState = false;
                    }
                    break;
                case MOVING_TO_FRONT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) || skipCurrentPassState) {
                        passThruTimer.reset();
                        clawIsPass = false;
                        currentPassState = passStates.IN_FRONT;
                        passIsFront = true;
                        skipCurrentPassState = false;
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

    public void runLiftToPos() {
        liftController.setSetPoint(targetLiftPos);

        if (useLiftPIDF && !liftController.atSetPoint()) {
            liftVelocity = liftController.calculate(lift_motor2.getCurrentPosition());

            if (liftVelocity < TeleOpConfig.LIFT_MAX_DOWN_VELOCITY) {
                liftVelocity = TeleOpConfig.LIFT_MAX_DOWN_VELOCITY;
            }

            if (liftVelocity < 0.2 &&
                !passIsLifted &&
                dropClawTimer.seconds() >= TeleOpConfig.DROP_TO_RETRACT_TIME &&
                (currentPassState == passStates.IN_FRONT || currentPassState == passStates.IN_BACK)
               ) {
                lastPassPos = currentPassPos;
                currentPassPos = passPositions.UP;
                passIsLifted = true;
                clawIsPass = true;
            }
            if (passIsLifted && liftVelocity >= 0) {
                currentPassPos = lastPassPos;
                passIsLifted = false;
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

    public void runClaw () {
        if (!clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_CLOSED_ANGLE);
        } else if (clawIsPass) {
            clawRight.turnToAngle(TeleOpConfig.CLAW_PASS_ANGLE);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_OPEN_ANGLE);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !clawHasLifted) {
            targetLiftPos = liftController.getSetPoint() + 150;
            liftClawTimer.reset();
            clawHasLifted = true;
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
        clawHasLifted = false;
    }

    public void dropClaw () {
        if (!clawIsOpen) {
            dropClawTimer.reset();
        }
        clawIsOpen = true;
        clawHasDropped = false;
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
            skipCurrentPassState = true;
        } else {
            passIsLifted = false;
            clawIsPass = true;
            passThruTimer.reset();
            currentPassState = passStates.MOVING_TO_PIVOT;
            currentPassPos = passPositions.PIVOT_POS;
            skipCurrentPassState = false;
        }
    }
}
