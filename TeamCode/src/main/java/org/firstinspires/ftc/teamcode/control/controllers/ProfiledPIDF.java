package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledPIDF extends PIDFController {

    protected MotionProfile profile;

    private MotionState profileState = new MotionState(0.0, 0.0, 0.0, 0.0);

    private final ElapsedTime profileTimer = new ElapsedTime();

    private double MAX_VELO = 1.0;
    private double MAX_ACCEL = 1.0;
    private double MAX_JERK = 0.0;

    /**
     * Initialize fields <p>
     * Use {@link #updateConstraints} to update constants
     *
     * @param pid         PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public ProfiledPIDF(PIDController pid, FeedforwardController feedforward) {
        super(pid, feedforward);
        profileTimer.reset();
    }

    public ProfiledPIDF() {
        this(new PIDController(), new FeedforwardController());
    }

    public void setTargetPosition(double currentPosition, double currentVelocity, double targetPosition) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPosition, currentVelocity),
                new MotionState(targetPosition, 0.0),
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );
        profileTimer.reset();
    }

    public void setTargetPosition(double currentPosition, double targetPosition) {
        setTargetPosition(currentPosition, 0.0, targetPosition);
    }

    public void updateConstraints(double MAX_VELO, double MAX_ACCEL, double MAX_JERK) {
        this.MAX_VELO = MAX_VELO;
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_JERK = MAX_JERK;
    }

    public void updateConstraints(double MAX_VELO, double MAX_ACCEL) {
        updateConstraints(MAX_VELO, MAX_ACCEL, MAX_JERK);
    }

    @Override
    public double update(double currentPosition, double voltage) {
        profileState = profile.get(profileTimer.seconds());
        setTargetState(profileState.getX(), profileState.getV(), profileState.getA());
        return super.update(currentPosition, voltage);
    }

    @Override
    public double update(double currentPosition) {
        return this.update(currentPosition, 12.0);
    }

    public double getProfilePosition() {
        return profileState.getX();
    }

    public double getProfileVelocity() {
        return profileState.getV();
    }

    public double getProfileAcceleration() {
        return profileState.getA();
    }

    public double getProfileJerk() {
        return profileState.getJ();
    }

    public boolean isDone() {
        return profileTimer.seconds() > profile.duration();
    }
}
