package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.subsystems.SimplePivot;

public class PowerplayPassthrough extends ProfiledClawArm {

    public static SimpleServo axon(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    public PowerplayPassthrough(HardwareMap hw) {
        super(
                new SimplePivot(
                        axon(hw, "claw right"),
                        RobotConfig.ANGLE_CLAW_OPEN,
                        RobotConfig.ANGLE_CLAW_CLOSED
                ),
                new SimplePivot(
                        RobotConfig.reverseServo(axon(hw, "claw pivot")),
                        RobotConfig.ANGLE_PIVOT_FRONT,
                        RobotConfig.ANGLE_PIVOT_BACK
                ),
                new SimpleServo[]{
                        axon(hw, "passthrough 1"),
                        RobotConfig.reverseServo(axon(hw, "passthrough 2"))
                }
        );
        updateConstants();
        currentAngle = RobotConfig.ANGLE_PASS_FRONT;
    }

    @Override
    public void run() {
        updateConstants();
        super.run();
    }

    private void updateConstants() {
        updateConstants(
                RobotConfig.ANGLE_PASS_FRONT,
                RobotConfig.ANGLE_PASS_BACK,
                RobotConfig.ANGLE_PASS_TILT_OFFSET,
                RobotConfig.ANGLE_PASS_MINI_TILT_OFFSET,
                RobotConfig.ANGLE_PIVOT_POS,
                RobotConfig.PASS_PIVOT_POS_TOLERANCE,
                RobotConfig.PASS_MAX_VELO,
                RobotConfig.PASS_MAX_ACCEL,
                RobotConfig.PASS_MAX_JERK
        );
        claw.updateAngles(RobotConfig.ANGLE_CLAW_OPEN, RobotConfig.ANGLE_CLAW_CLOSED);
        wrist.updateAngles(RobotConfig.ANGLE_PIVOT_FRONT, RobotConfig.ANGLE_PIVOT_BACK);
    }
}
