package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public DigitalChannel limitSwitch;
    public static ElapsedTime passThruTimer;
    public static ElapsedTime liftClawTimer;

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


    public boolean clawIsOpen = true;
    public boolean clawPass = false;

    public boolean passIsFront = true;
    public boolean pivotIsFront = true;


    public enum passPositions {
        MOVING_TO_FRONT, FRONT, CLAW_CLOSING, MOVING_TO_PIVOT, PIVOT, MOVING_TO_BACK, BACK
    }

    passPositions currentPos = passPositions.FRONT;

    public void runPassthrough () {
        if (passIsFront) {
            switch (currentPos) {
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
                        currentPos = passPositions.MOVING_TO_PIVOT;
                    }
                    break;
                case MOVING_TO_PIVOT:
                    if (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT) {
                        passThruTimer.reset();
                        currentPos = passPositions.PIVOT;
                        togglePivot();
                    } else {
                        passThruRight.turnToAngle(TeleOpConfig.PASS_1_PIVOTING);
                        passThruLeft.turnToAngle(TeleOpConfig.PASS_2_PIVOTING);
                    }

                    if (passThruTimer.seconds() >= TeleOpConfig.FRONT_TO_PIVOT_TIME) {
                        passThruTimer.reset();
                        currentPos = passPositions.PIVOT;
                        togglePivot();
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
                        currentPos = passPositions.MOVING_TO_BACK;
                    }
                    break;
                case MOVING_TO_BACK:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_BACK);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_BACK);

                    if (passThruTimer.seconds() >= (TeleOpConfig.PIVOT_TO_BACK_TIME)) {
                        passThruTimer.reset();
                        clawPass = false;
                        currentPos = passPositions.BACK;
                        passIsFront = false;
                    }
                    break;
                case BACK:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_BACK);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_BACK);
                    passThruTimer.reset();
                    break;
                default:
                    currentPos = passPositions.FRONT;
                    break;
            }
        } else {
            switch (currentPos) {
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
                        currentPos = passPositions.MOVING_TO_PIVOT;
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
                        currentPos = passPositions.PIVOT;
                        togglePivot();
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
                        currentPos = passPositions.MOVING_TO_FRONT;
                    }
                    break;
                case MOVING_TO_FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
                    if ((passThruTimer.seconds() >= TeleOpConfig.PIVOT_TO_FRONT_TIME) || (lift_motor2.encoder.getPosition() >= TeleOpConfig.MINIMUM_PIVOT_HEIGHT)) {
                        passThruTimer.reset();
                        clawPass = false;
                        currentPos = passPositions.FRONT;
                        passIsFront = true;
                    }
                    break;
                case FRONT:
                    passThruRight.turnToAngle(TeleOpConfig.PASS_1_FRONT);
                    passThruLeft.turnToAngle(TeleOpConfig.PASS_2_FRONT);
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
                liftPosStr = heightVal.ONE.name();
                break;
            case TWO:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_TWO);
                liftPosStr = heightVal.TWO.name();
                break;
            case THREE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_THREE);
                liftPosStr = heightVal.THREE.name();
                break;
            case FOUR:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_FOUR);
                liftPosStr = heightVal.FOUR.name();
                break;
            case FIVE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_FIVE);
                liftPosStr = heightVal.FIVE.name();
                break;
            case GROUND:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_GROUND);
                liftPosStr = heightVal.GROUND.name();
                break;
            case LOW:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_LOW);
                liftPosStr = heightVal.LOW.name();
                break;
            case MED:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_MEDIUM);
                liftPosStr = heightVal.MED.name();
                break;
            case TALL:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_TALL);
                liftPosStr = heightVal.TALL.name();
                break;
        }
    }


    public void runLiftPos() {
        if (!liftController.atSetPoint()) {
            double velocity = liftController.calculate(
                lift_motor2.getCurrentPosition()
            );

            if (velocity < TeleOpConfig.LIFT_MAX_DOWN_VELOCITY) {
                velocity = TeleOpConfig.LIFT_MAX_DOWN_VELOCITY;
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
        if (clawPass) {
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_PASS);
        } else if (clawIsOpen){
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_OPEN);
        } else {
            clawRight.turnToAngle(TeleOpConfig.CLAW_RIGHT_CLOSED);
        }
    }

    public void liftClaw () {
        clawIsOpen = false;
        liftClawTimer.reset();
        if (liftClawTimer.seconds() >= TeleOpConfig.CLAW_CLOSING_TIME) {
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
            clawPivot.turnToAngle(TeleOpConfig.PIVOT_FRONT);
        } else {
            clawPivot.turnToAngle(TeleOpConfig.PIVOT_BACK);
        }
    }

    public void togglePassthrough () {
        if ((currentPos != passPositions.FRONT) && (currentPos !=passPositions.BACK)) {
            passIsFront = !passIsFront;

        } else {
            if (!clawIsOpen) {
                currentPos = passPositions.MOVING_TO_PIVOT;
            } else {
                clawPass = true;
                currentPos = passPositions.CLAW_CLOSING;
                passThruTimer.reset();
            }
        }
    }
}
