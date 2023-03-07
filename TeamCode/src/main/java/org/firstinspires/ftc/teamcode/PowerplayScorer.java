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
    private SimpleServo passThruRight;
    private SimpleServo passThruLeft;
    private PIDFController liftController;
    public String targetLiftPosName;
    public DigitalChannel limitSwitch;
    public DigitalChannel red1;
    public DigitalChannel green1;
    public DigitalChannel red2;
    public DigitalChannel green2;
    public double targetLiftHeight;
    public double liftVelocity;
    private static ElapsedTime passThruTimer;
    private static ElapsedTime liftClawTimer;

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
        targetLiftHeight = TeleOpConfig.HEIGHT_ONE;
        targetLiftPosName = liftHeights.ONE.name();

        passThruTimer = new ElapsedTime();
        passThruTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();

        limitSwitch = hw.get(DigitalChannel.class, "limit switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        red1 = hw.get(DigitalChannel.class, "red1");
        green1 = hw.get(DigitalChannel.class, "green1");
        red1.setMode(DigitalChannel.Mode.OUTPUT);
        green1.setMode(DigitalChannel.Mode.OUTPUT);

        red2 = hw.get(DigitalChannel.class, "red2");
        green2 = hw.get(DigitalChannel.class, "green2");
        red2.setMode(DigitalChannel.Mode.OUTPUT);
        green2.setMode(DigitalChannel.Mode.OUTPUT);

    }

    //  lift motor encoder resolution (ticks):
    private static final double LIFT_TICKS = 145.1;

    public boolean clawIsOpen = true;
    public boolean clawIsPass = false;
    public boolean passIsFront = true;
    private boolean pivotIsFront = true;
    // override variable--when true, skips the timer to switch to next state immediately
    private boolean skip = false;
    public boolean useLiftPIDF = true;

    public double currentLiftHeight;

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
                if (currentLiftHeight >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                    passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_FRONT_ANGLE);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_FRONT_ANGLE);
                } else {
                    passThruRight.turnToAngle(TeleOpConfig.PASS_RIGHT_PIVOT_ANGLE);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_LEFT_PIVOT_ANGLE);
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
                    skip = false;
                    break;
                case MOVING_TO_FRONT:
                    passThruTimer.reset();
                    currentPassState = passStates.MOVING_TO_PIVOT;
                    currentPassPos = passPositions.PIVOT_POS;
                    skip = false;
                    break;
                case MOVING_TO_PIVOT:
                    if ((passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) || skip || currentLiftHeight >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
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
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (currentLiftHeight >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) || skip) {
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
                targetLiftHeight = TeleOpConfig.HEIGHT_ONE;
                targetLiftPosName = liftHeights.ONE.name();
                break;
            case TWO:
                targetLiftHeight = TeleOpConfig.HEIGHT_TWO;
                targetLiftPosName = liftHeights.TWO.name();
                break;
            case THREE:
                targetLiftHeight = TeleOpConfig.HEIGHT_THREE;
                targetLiftPosName = liftHeights.THREE.name();
                break;
            case FOUR:
                targetLiftHeight = TeleOpConfig.HEIGHT_FOUR;
                targetLiftPosName = liftHeights.FOUR.name();
                break;
            case FIVE:
                targetLiftHeight = TeleOpConfig.HEIGHT_FIVE;
                targetLiftPosName = liftHeights.FIVE.name();
                break;
            case GROUND:
                targetLiftHeight = TeleOpConfig.HEIGHT_GROUND;
                targetLiftPosName = liftHeights.GROUND.name();
                break;
            case LOW:
                targetLiftHeight = TeleOpConfig.HEIGHT_LOW;
                targetLiftPosName = liftHeights.LOW.name();
                break;
            case MED:
                targetLiftHeight = TeleOpConfig.HEIGHT_MEDIUM;
                targetLiftPosName = liftHeights.MED.name();
                break;
            case TALL:
                targetLiftHeight = TeleOpConfig.HEIGHT_TALL;
                targetLiftPosName = liftHeights.TALL.name();
                break;
        }
    }

    public void runLiftToPos() {
        currentLiftHeight = lift_motor2.encoder.getPosition() * TeleOpConfig.LIFT_TICKS_PER_INCH;
        liftController.setSetPoint(targetLiftHeight);

        if (useLiftPIDF && !liftController.atSetPoint()) {
            liftVelocity = liftController.calculate(currentLiftHeight);

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
        currentLiftHeight = 0;
    }

    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    private boolean hasLifted = true;

    public void runClaw () {
        if (!clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_CLOSED_ANGLE);
        } else if (clawIsPass) {
            clawRight.turnToAngle(TeleOpConfig.CLAW_PASS_ANGLE);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_OPEN_ANGLE);
        }


        if ((liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) && !hasLifted) {
            targetLiftHeight = liftController.getSetPoint() + 5;
            liftClawTimer.reset();
            hasLifted = true;
        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        hasLifted = false;
    }

    public void dropClaw () {
        targetLiftHeight = TeleOpConfig.HEIGHT_ONE;
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
