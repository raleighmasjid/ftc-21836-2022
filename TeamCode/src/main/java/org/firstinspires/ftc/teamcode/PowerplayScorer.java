package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    public String liftPosStr;
    public TouchSensor limitSwitch;
    public ElapsedTime passThruTimer;
    public ElapsedTime liftClawTimer;

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
        liftController.setTolerance(TeleOpConfig.LIFT_E_TOLERANCE, TeleOpConfig.LIFT_V_TOLERANCE);

        lift_motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lift_motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        lift_motor1.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor2.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor3.setRunMode(Motor.RunMode.VelocityControl);

        lift_motor1.setInverted(true);
        lift_motor3.setInverted(true);

        liftPosStr = null;

        passThruTimer = new ElapsedTime();
        passThruTimer.reset();
        liftClawTimer = new ElapsedTime();
        liftClawTimer.reset();
    }


    //  lift motor encoder resolution (ticks):
    public static final double LIFT_TICKS = 145.1;

    // states that the claw should be open upon teleop control loop start
    public boolean clawIsOpen = true;
    public boolean clawWasOpen = true;
    public boolean passIsFront = true;
    public boolean pivotIsFront = true;


    public enum passPositions {
        MOVING_TO_FRONT, FRONT, CLAW_CLOSING, MOVING_TO_PIVOT, PIVOT, MOVING_TO_BACK, BACK, OVERRIDE
    }

    passPositions currentPos = passPositions.FRONT;

    public void runPassthrough () {
        if (passIsFront) {
            switch (currentPos) {
                case FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Front);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Front);
                    passThruTimer.reset();
                    break;
                case CLAW_CLOSING:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Front);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Front);
                    if (passThruTimer.seconds() >= TeleOpConfig.clawClosingTime) {
                        passThruTimer.reset();
                        currentPos = passPositions.MOVING_TO_PIVOT;
                    }
                    break;
                case MOVING_TO_PIVOT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Pivoting);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Pivoting);
                    if (passThruTimer.seconds() >= TeleOpConfig.frontToPivot) {
                        passThruTimer.reset();
                        currentPos = passPositions.PIVOT;
                        togglePivot();
                    }
                    break;
                case PIVOT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Pivoting);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Pivoting);
                    if (passThruTimer.seconds() >= TeleOpConfig.pivotingTime) {
                        passThruTimer.reset();
                        currentPos = passPositions.MOVING_TO_BACK;
                    }
                    break;
                case MOVING_TO_BACK:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Back);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Back);
                    if (passThruTimer.seconds() >= (TeleOpConfig.backToPivot/1.6)) {
                        passThruTimer.reset();
                        clawIsOpen = clawWasOpen;
                        currentPos = passPositions.BACK;
                        passIsFront = false;
                    }
                    break;
                case BACK:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Back);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Back);
                    passThruTimer.reset();
                    break;
                default:
                    currentPos = passPositions.FRONT;
                    break;
            }
        } else {
            switch (currentPos) {
                case BACK:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Back);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Back);
                    passThruTimer.reset();
                    break;
                case CLAW_CLOSING:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Back);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Back);
                    if (passThruTimer.seconds() >= TeleOpConfig.clawClosingTime) {
                        passThruTimer.reset();
                        currentPos = passPositions.MOVING_TO_PIVOT;
                    }
                    break;
                case MOVING_TO_PIVOT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Pivoting);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Pivoting);
                    if (passThruTimer.seconds() >= TeleOpConfig.backToPivot) {
                        passThruTimer.reset();
                        currentPos = passPositions.PIVOT;
                        togglePivot();
                    }
                    break;
                case PIVOT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Pivoting);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Pivoting);
                    if (passThruTimer.seconds() >= TeleOpConfig.pivotingTime) {
                        passThruTimer.reset();
                        currentPos = passPositions.MOVING_TO_FRONT;
                    }
                    break;
                case MOVING_TO_FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Front);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Front);
                    if (passThruTimer.seconds() >= TeleOpConfig.frontToPivot) {
                        passThruTimer.reset();
                        clawIsOpen = clawWasOpen;
                        currentPos = passPositions.FRONT;
                        passIsFront = true;
                    }
                    break;
                case FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.pass1Front);
                    passThruLeft.turnToAngle(TeleOpConfig.pass2Front);
                    passThruTimer.reset();
                    break;
                default:
                    currentPos = passPositions.FRONT;
                    break;
            }
        }
    }


    public enum heightVal {
            ONE, TWO, THREE, FOUR, FIVE, GROUND, LOW, MED, TALL
    }

    public void setLiftPos(heightVal height) {

        switch (height){
            case ONE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_ONE);
                liftPosStr = "floor / stack of one";
                break;
            case TWO:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_TWO);
                liftPosStr = "stack of 2";
                break;
            case THREE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_THREE);
                liftPosStr = "stack of 3";
                break;
            case FOUR:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_FOUR);
                liftPosStr = "stack of 4";
                break;
            case FIVE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_FIVE);
                liftPosStr = "stack of 5";
                break;
            case GROUND:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_GROUND);
                liftPosStr = "ground junction height";
                break;
            case LOW:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_LOW);
                liftPosStr = "low pole height";
                break;
            case MED:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_MEDIUM);
                liftPosStr = "medium pole height";
                break;
            case TALL:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_TALL);
                liftPosStr = "tall pole height";
                break;
        }
    }


    public void runLiftPos() {
        if (!liftController.atSetPoint()) {
            double velocity = liftController.calculate(
                lift_motor2.getCurrentPosition()
            );

            if (velocity < TeleOpConfig.liftDownMaxVelo) {
                velocity = TeleOpConfig.liftDownMaxVelo;
            }

            runLift(velocity);
        }
    }

    // takes an analog stick input (-1 to 1)
    public void runLift(double velocity) {
        // squares input but keeps +/- sign
//        velocity = signSquare(velocity);
        // sets both lift motor power to squared value
        // this allows for smoother acceleration
        lift_motor1.set(velocity);
        lift_motor2.set(velocity);
        lift_motor3.set(velocity);
    }


    public void toggleClaw () {
        clawIsOpen = !clawIsOpen;
    }

    public void runClaw () {

        if (clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_OPEN);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_CLOSED);
        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        if (liftClawTimer.seconds() >= TeleOpConfig.clawClosingTime) {
            liftController.setSetPoint(TeleOpConfig.HEIGHT_GROUND);
        }
    }
    public void dropClaw () {
        liftController.setSetPoint(TeleOpConfig.HEIGHT_ONE);
        clawIsOpen = true;
    }


    public void togglePivot () {
        pivotIsFront = !pivotIsFront;
    }

    public void runPivot () {
        if(pivotIsFront) {
            clawPivot.turnToAngle(TeleOpConfig.pivotFront);
        } else {
            clawPivot.turnToAngle(TeleOpConfig.pivotBack);
        }
    }

    public void togglePassthrough () {
        if ((currentPos != passPositions.FRONT) && (currentPos !=passPositions.BACK)) {
            passIsFront = !passIsFront;

        } else {

            clawWasOpen = clawIsOpen;
            if (!clawIsOpen) {
                currentPos = passPositions.MOVING_TO_PIVOT;
            } else {
                clawIsOpen = false;
                currentPos = passPositions.CLAW_CLOSING;
                passThruTimer.reset();
            }
        }
    }
}
