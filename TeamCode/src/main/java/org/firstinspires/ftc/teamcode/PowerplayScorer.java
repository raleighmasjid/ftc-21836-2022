package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Creates a 'GreenBot' class that extends the mecanum drive class
 * @author Arshad Anas
 * @since 2022/12/24
 */

public class PowerplayScorer {

    public MotorEx lift_motor1;
    public MotorEx lift_motor2;
//    public MotorEx lift_motor3;
    public SimpleServo clawLeft;
    public SimpleServo clawRight;
    public PIDFController liftController;
    public String liftPos;

    // the following is the code that runs during initialization
    public void init(HardwareMap hw) {

        clawLeft = new SimpleServo(hw,"claw left",0,180);
        clawRight = new SimpleServo(hw,"claw right",0,180);

        lift_motor1 = new MotorEx(hw, "lift motor 1", LIFT_TICKS, MAX_RPM);
        lift_motor2 = new MotorEx(hw, "lift motor 2", LIFT_TICKS, MAX_RPM);
//        lift_motor3 = new MotorEx(hw, "lift motor 3", LIFT_TICKS, MAX_RPM);

        liftController = new PIDFController(
                TeleOpConfig.LIFT_P,
                TeleOpConfig.LIFT_I,
                TeleOpConfig.LIFT_D,
                TeleOpConfig.LIFT_F
        );
        liftController.setTolerance(TeleOpConfig.LIFT_E_TOLERANCE, TeleOpConfig.LIFT_V_TOLERANCE);

        lift_motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lift_motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lift_motor1.setRunMode(Motor.RunMode.VelocityControl);
        lift_motor2.setRunMode(Motor.RunMode.VelocityControl);
//        lift_motor3.setRunMode(Motor.RunMode.VelocityControl);
//        lift_motor3.setInverted(true);

        liftPos = null;

    }
    //  lift motor encoder resolution (ticks):
    public static final double LIFT_TICKS = 145.1;

    // states that the claw should be open upon teleop control loop start
    public boolean clawOpen = true;

    // squares input but keeps +/- sign
    public double signSquare (double x) {
        return x * Math.abs(x);
    }

    public enum HEIGHT_VAL {
            ONE, TWO, THREE, FOUR, FIVE, GROUND, LOW, MED, TALL
    }

    public void setLiftPos(HEIGHT_VAL height) {
        switch (height){
            case ONE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_ONE);
                liftPos = "floor / stack of one";
                break;
            case TWO:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_TWO);
                liftPos = "stack of 2";
                break;
            case THREE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_THREE);
                liftPos = "stack of 3";
                break;
            case FOUR:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_FOUR);
                liftPos = "stack of 4";
                break;
            case FIVE:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_FIVE);
                liftPos = "stack of 5";
                break;
            case GROUND:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_GROUND);
                liftPos = "ground junction height";
                break;
            case LOW:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_LOW);
                liftPos = "low pole height";
                break;
            case MED:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_MEDIUM);
                liftPos = "medium pole height";
                break;
            case TALL:
                liftController.setSetPoint(TeleOpConfig.HEIGHT_TALL);
                liftPos = "tall pole height";
                break;
        }
    }

    public void runLiftPos() {
        if (!liftController.atSetPoint()) {
            double velocity = liftController.calculate(
                lift_motor1.getCurrentPosition()
            );
            runLift(velocity);
        }
    }

    // takes an analog stick input (-1 to 1)
    public void runLift(double velocity) {
        // squares input but keeps +/- sign
        velocity = signSquare(velocity);
        // sets both lift motor power to squared value
        // this allows for smoother acceleration
        lift_motor1.set(velocity);
        lift_motor2.set(velocity);
//        lift_motor3.set(velocity);
    }

    // inverts claw state boolean
    public void toggleClaw () {
        clawOpen = !clawOpen;
    }

    // the following code switches the open/closed state of the claw based on the boolean above
    public void runClaw() {
        if (clawOpen){
            clawLeft.setPosition(TeleOpConfig.CLAW_LEFT_OPEN);
            clawRight.setPosition(TeleOpConfig.CLAW_RIGHT_OPEN);
        } else {
            clawLeft.setPosition(TeleOpConfig.CLAW_LEFT_CLOSED);
            clawRight.setPosition(TeleOpConfig.CLAW_RIGHT_CLOSED);
        }
    }

    public void liftClaw() {
        clawOpen = false;
        liftController.setSetPoint(TeleOpConfig.HEIGHT_GROUND);
    }
    public void dropClaw() {
        liftController.setSetPoint(TeleOpConfig.HEIGHT_ONE);
        clawOpen = true;
    }

}
