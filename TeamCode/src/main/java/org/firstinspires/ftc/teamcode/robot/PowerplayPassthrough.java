package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.systems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.systems.SimpleClaw;

public class PowerplayPassthrough extends ProfiledClawArm {

    public PowerplayPassthrough(SimpleClaw claw, SimpleServo pivotServo, SimpleServo servoR, SimpleServo servoL) {
        super(claw, pivotServo, servoR, servoL);
        updateConstants();
        currentAngle = ANGLE_FRONT;
    }

    @Override
    public void run() {
        updateConstants();
        super.run();
    }

    protected void updateConstants() {
        updateConstants(
                RobotConfig.ANGLE_PASS_FRONT,
                RobotConfig.ANGLE_PASS_BACK,
                RobotConfig.ANGLE_PIVOT_FRONT,
                RobotConfig.ANGLE_PIVOT_BACK,
                RobotConfig.ANGLE_PASS_TILT_OFFSET,
                RobotConfig.ANGLE_PASS_MINI_TILT_OFFSET,
                RobotConfig.ANGLE_PIVOT_POS,
                RobotConfig.PASS_PIVOT_POS_TOLERANCE,
                RobotConfig.PASS_MAX_VELO,
                RobotConfig.PASS_MAX_ACCEL,
                RobotConfig.PASS_MAX_JERK
        );
        claw.updateAngles(RobotConfig.ANGLE_CLAW_OPEN, RobotConfig.ANGLE_CLAW_CLOSED);
    }
}
