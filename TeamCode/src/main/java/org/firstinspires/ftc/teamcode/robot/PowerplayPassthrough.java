package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.subsystems.SimplePivot;

public class PowerplayPassthrough extends ProfiledClawArm {

    private boolean tilted = false;

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

    /**
     * Toggles the value of {@link #tilted}
     */
    public void toggleTilt() {
        setTilted(!tilted);
    }

    /**
     * Sets the value of {@link #tilted} and runs {@link #updateProfile}
     */
    public void setTilted(boolean tilted) {
        this.tilted = tilted;
        updateProfile();
    }

    @Override
    protected void updateProfile() {
        double tiltOffset =
                tilted ?
                        RobotConfig.ANGLE_PASS_TILT_OFFSET :
                        (!triggered) && (inBack != wrist.getActivated()) ? RobotConfig.ANGLE_PASS_MINI_TILT_OFFSET : 0.0;

        ANGLE_FRONT = RobotConfig.ANGLE_PASS_FRONT + tiltOffset;
        ANGLE_BACK = RobotConfig.ANGLE_PASS_BACK - tiltOffset;

        super.updateProfile();
    }

    @Override
    public void reset() {
        super.reset();
        setTilted(false);
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
