package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.systems.SimpleClaw;

public class PowerplayPassthrough extends ProfiledClawArm {

    public static SimpleServo getAxon(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    public PowerplayPassthrough(HardwareMap hw) {
        super(
                new SimpleClaw(getAxon(hw, "claw right"), RobotConfig.ANGLE_CLAW_OPEN, RobotConfig.ANGLE_CLAW_CLOSED),
                PowerplayScorer.getReversedServo(getAxon(hw, "claw pivot")),
                getAxon(hw, "passthrough 1"),
                PowerplayScorer.getReversedServo(getAxon(hw, "passthrough 2"))
        );
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
