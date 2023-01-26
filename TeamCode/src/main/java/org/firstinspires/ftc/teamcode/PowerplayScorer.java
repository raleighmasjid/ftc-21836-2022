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
 * Creates a 'GreenBot' class that extends the mecanum drive class
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

    // the following is the code that runs during initialization
    public void init(HardwareMap hw) {

        clawRight = new SimpleServo(hw,"claw right",0,300);
        clawPivot = new SimpleServo(hw, "claw pivot",0,300);
        passThruRight = new SimpleServo(hw, "passthrough 1",0,300);
        passThruLeft = new SimpleServo(hw, "passthrough 2",0,300);

        lift_motor1 = new MotorEx(hw, "lift motor 1", LIFT_TICKS, MAX_RPM);
        lift_motor2 = new MotorEx(hw, "lift motor 2", LIFT_TICKS, MAX_RPM);
        lift_motor3 = new MotorEx(hw, "lift motor 3", LIFT_TICKS, MAX_RPM);

        liftController.setTolerance(TeleOpConfig.LIFT_E_TOLERANCE, TeleOpConfig.LIFT_V_TOLERANCE);
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

        limitSwitch = hw.get(DigitalChannel.class, "limit switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }


    public double clip (double input, double min, double max) {
        return Math.min(Math.max(input, min), max);
    }

    //  lift motor encoder resolution (ticks):
    public static final double LIFT_TICKS = 145.1;


    public boolean clawIsOpen = true;
    public boolean clawIsPass = false;

    public boolean passIsFront = true;
    public boolean pivotIsFront = true;


    public enum passPositions {
        MOVING_TO_FRONT, FRONT, CLAW_CLOSING, MOVING_TO_PIVOT, PIVOT, MOVING_TO_BACK, BACK
    }

    passPositions currentPassPos = passPositions.FRONT;

    public void runPassthrough () {
        if (passIsFront) {
            switch (currentPassPos) {
                case FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    passThruTimer.reset();
                    break;
                case CLAW_CLOSING:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    if (passThruTimer.seconds() >= TeleOpConfig.CLAW_PASS_CLOSING_TIME) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.MOVING_TO_PIVOT;
                    }
                    break;
                case MOVING_TO_PIVOT:
                    if (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.PIVOT;
                        pivotIsFront = false;
                    } else {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_PIVOTING);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_PIVOTING);
                    }

                    if (passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.PIVOT;
                        pivotIsFront = false;
                    }
                    break;
                case PIVOT:
                    if (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    } else {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_PIVOTING);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_PIVOTING);
                    }

                    if (passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_BACK_TIME) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.MOVING_TO_BACK;
                    }
                    break;
                case MOVING_TO_BACK:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_BACK);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_BACK);

                    if (passThruTimer.seconds() >= (TeleOpConfig.PIVOT_TO_BACK_TIME)) {
                        passThruTimer.reset();
                        clawIsPass = false;
                        currentPassPos = passPositions.BACK;
                        passIsFront = false;
                    }
                    break;
                case BACK:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_BACK);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_BACK);
                    passThruTimer.reset();
                    break;
                default:
                    currentPassPos = passPositions.FRONT;
                    break;
            }
        } else {
            switch (currentPassPos) {
                case BACK:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_BACK);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_BACK);
                    passThruTimer.reset();
                    break;
                case CLAW_CLOSING:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_BACK);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_BACK);
                    if (passThruTimer.seconds() >= TeleOpConfig.CLAW_PASS_CLOSING_TIME) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.MOVING_TO_PIVOT;
                    }
                    break;
                case MOVING_TO_PIVOT:
                    if (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    } else {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_PIVOTING);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_PIVOTING);
                    }

                    if (passThruTimer.seconds() >= TeleOpConfig.BACK_TO_PIVOT_TIME) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.PIVOT;
                        pivotIsFront = true;
                    }
                    break;
                case PIVOT:
                    if (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    } else {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_PIVOTING);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_PIVOTING);
                    }

                    if (passThruTimer.seconds() >= TeleOpConfig.PIVOTING_TO_FRONT_TIME) {
                        passThruTimer.reset();
                        currentPassPos = passPositions.MOVING_TO_FRONT;
                    }
                    break;
                case MOVING_TO_FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT)) {
                        passThruTimer.reset();
                        clawIsPass = false;
                        currentPassPos = passPositions.FRONT;
                        passIsFront = true;
                    }
                    break;
                case FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    passThruTimer.reset();
                    break;
                default:
                    currentPassPos = passPositions.FRONT;
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

        if (!liftController.atSetPoint()) {
            liftVelocity = liftController.calculate(lift_motor2.getCurrentPosition());

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

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    public boolean hasDropped = true;

    public boolean hasLifted = true;

    public void runClaw () {
        if (clawIsPass) {
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_PASS);
        } else if (clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_OPEN);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_CLOSED);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !hasLifted) {
            targetLiftPos = liftController.getSetPoint() + 80;
            liftController.setSetPoint(targetLiftPos);
            liftClawTimer.reset();
            hasDropped = true;
        }
        if ((dropClawTimer.seconds() >= TeleOpConfig.CLAW_OPEN_TO_DROP_TIME) && !hasDropped) {
            liftController.setSetPoint(TeleOpConfig.HEIGHT_ONE);
            dropClawTimer.reset();
            hasDropped = true;
        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        hasLifted = false;
    }

    public void dropClaw () {
        clawIsOpen = true;
        dropClawTimer.reset();
        hasDropped = false;
    }


    public void togglePivot () {
        pivotIsFront = !pivotIsFront;
    }

    public void runPivot () {
        if(pivotIsFront) {
            clawPivot.turnToAngle(TeleOpConfig.PIVOT_FRONT);
        } else {
            clawPivot.turnToAngle(TeleOpConfig.PIVOT_BACK);
        }
    }

    public void togglePassthrough () {
        if ((currentPassPos != passPositions.FRONT) && (currentPassPos !=passPositions.BACK)) {
            passIsFront = !passIsFront;

        } else {
            if (!clawIsOpen) {
                currentPassPos = passPositions.MOVING_TO_PIVOT;
            } else {
                clawIsPass = true;
                currentPassPos = passPositions.CLAW_CLOSING;
                passThruTimer.reset();
            }
        }
    }
}
