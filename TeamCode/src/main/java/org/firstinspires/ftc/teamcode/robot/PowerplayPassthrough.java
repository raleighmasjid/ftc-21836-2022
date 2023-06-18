package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.systems.SimpleClaw;

public class PowerplayPassthrough extends ProfiledClawArm {

    public PowerplayPassthrough(HardwareMap hw) {
        super(
                new SimpleClaw(PowerplayScorer.getAxon(hw, "claw right"), RobotConfig.ANGLE_CLAW_OPEN, RobotConfig.ANGLE_CLAW_CLOSED),
                PowerplayScorer.getReversedServo(PowerplayScorer.getAxon(hw, "claw pivot")),
                PowerplayScorer.getAxon(hw, "passthrough 1"),
                PowerplayScorer.getReversedServo(PowerplayScorer.getAxon(hw, "passthrough 2"))
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
