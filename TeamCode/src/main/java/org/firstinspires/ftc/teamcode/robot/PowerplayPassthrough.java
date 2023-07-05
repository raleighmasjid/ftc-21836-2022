package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ProfiledClawArm;
import org.firstinspires.ftc.teamcode.subsystems.SimplePivot;

@Config
public class PowerplayPassthrough extends ProfiledClawArm {

    public static double
            ANGLE_CLAW_CLOSED = 0.0,
            ANGLE_CLAW_OPEN = 62.0,
            ANGLE_PIVOT_FRONT = 17.0,
            ANGLE_PIVOT_BACK = 216.0,
            ANGLE_PASS_FRONT = 8.0,
            ANGLE_PASS_TILT_OFFSET = 45.0,
            ANGLE_PASS_BACK = 310.0,
            ANGLE_PASS_MINI_TILT_OFFSET = 17.0,
            ANGLE_PIVOT_POS = (ANGLE_PASS_BACK - ANGLE_PASS_FRONT) * 0.5,
            PASS_PIVOT_POS_TOLERANCE = 30.0,
            PASS_MAX_VELO = 600.0,
            PASS_MAX_ACCEL = 3000.0,
            PASS_MAX_JERK = 7000.0;

    private boolean tilted = false;

    public final SimplePivot claw;

    public static SimpleServo axon(HardwareMap hw, String name) {
        return new SimpleServo(hw, name, 0, 355);
    }

    public static SimpleServo reverseServo(SimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public PowerplayPassthrough(HardwareMap hw) {
        super(
                new SimplePivot(
                        reverseServo(axon(hw, "claw pivot")),
                        ANGLE_PIVOT_FRONT,
                        ANGLE_PIVOT_BACK
                ),
                new SimpleServo[]{
                        axon(hw, "passthrough 1"),
                        reverseServo(axon(hw, "passthrough 2"))
                }
        );
        claw = new SimplePivot(
                axon(hw, "claw right"),
                ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED
        );
        updateConstants();
        currentAngle = ANGLE_PASS_FRONT;
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
                        ANGLE_PASS_TILT_OFFSET :
                        (!triggered) && (inBack != pivot.getActivated()) ? ANGLE_PASS_MINI_TILT_OFFSET : 0.0;

        ANGLE_FRONT = ANGLE_PASS_FRONT + tiltOffset;
        ANGLE_BACK = ANGLE_PASS_BACK - tiltOffset;

        super.updateProfile();
    }

    @Override
    public void reset() {
        super.reset();
        setTilted(false);
        claw.setActivated(false);
    }

    @Override
    public void run() {
        updateConstants();
        super.run();
        claw.run();
    }

    private void updateConstants() {
        updateConstants(
                ANGLE_PASS_FRONT,
                ANGLE_PASS_BACK,
                ANGLE_PIVOT_POS,
                PASS_PIVOT_POS_TOLERANCE,
                PASS_MAX_VELO,
                PASS_MAX_ACCEL,
                PASS_MAX_JERK
        );
        claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        pivot.updateAngles(ANGLE_PIVOT_FRONT, ANGLE_PIVOT_BACK);
    }

    @Override
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Claw is", claw.getActivated() ? "closed" : "open");
        telemetry.addLine();
        super.printTelemetry(telemetry);
    }
}
