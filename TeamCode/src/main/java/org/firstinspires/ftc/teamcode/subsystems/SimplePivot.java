package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.SimpleServo;

/**
 * Servo with two set positions <p>
 * Controlled by {@link #toggle} and {@link #setActivated}
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class SimplePivot {

    protected final SimpleServo servo;

    protected double ANGLE_A, ANGLE_B;

    protected boolean activated = false;

    /**
     * Initialize {@link SimplePivot}
     */
    public SimplePivot(SimpleServo servo, double ANGLE_A, double ANGLE_B) {
        this.servo = servo;
        updateAngles(ANGLE_A, ANGLE_B);
    }

    public void updateAngles(double ANGLE_A, double ANGLE_B) {
        this.ANGLE_A = ANGLE_A;
        this.ANGLE_B = ANGLE_B;
    }

    /**
     * Toggles the state of the {@link #servo}
     */
    public void toggle() {
        setActivated(!activated);
    }

    /**
     * Set state of the {@link #servo}
     *
     * @param activated False for position A, true for position B
     */
    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    /**
     * Get state of the {@link #servo} <p>
     * False if position A (default) <p>
     * True if in position B
     */
    public boolean getActivated() {
        return activated;
    }

    /**
     * Holds {@link #servo} position
     */
    public void run() {
        servo.turnToAngle(activated ? ANGLE_B : ANGLE_A);
    }
}
