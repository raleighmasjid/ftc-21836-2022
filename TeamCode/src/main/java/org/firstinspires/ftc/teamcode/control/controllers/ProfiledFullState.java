package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledFullState {

    protected MotionProfile profile;

    private MotionState profileState = new MotionState(0.0, 0.0, 0.0, 0.0);

    private final ElapsedTime profileTimer = new ElapsedTime();

    private double MAX_VELO = 1.0;
    private double MAX_ACCEL = 1.0;
    private double MAX_JERK = 0.0;

    public FullStateController fullState;
    public FeedforwardController feedforward;

    /**
     * Initialize fields <p>
     * Use {@link #updateConstraints} to update {@link #MAX_VELO}, {@link #MAX_ACCEL}, and {@link #MAX_JERK}
     */
    public ProfiledFullState(FullStateController fullState, FeedforwardController feedforward) {
        this.fullState = fullState;
        this.feedforward = feedforward;
        profileTimer.reset();
    }

    public ProfiledFullState() {
        this(new FullStateController(), new FeedforwardController());
    }

    public void setTargetPosition(double pCurrent, double vCurrent, double pTarget) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(pCurrent, vCurrent),
                new MotionState(pTarget, 0.0),
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );
        profileTimer.reset();
    }

    public void setTargetPosition(double pCurrent, double pTarget) {
        setTargetPosition(pCurrent, profileState.getV(), pTarget);
    }

    public void updateConstraints(double MAX_VELO, double MAX_ACCEL, double MAX_JERK) {
        this.MAX_VELO = MAX_VELO;
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_JERK = MAX_JERK;
    }

    public void updateConstraints(double MAX_VELO, double MAX_ACCEL) {
        updateConstraints(MAX_VELO, MAX_ACCEL, MAX_JERK);
    }

    public double update(double pCurrent, double vCurrent, double aCurrent, double voltage) {
        profileState = profile.get(profileTimer.seconds());

        fullState.setTargetState(profileState.getX(), profileState.getV(), profileState.getA());
        feedforward.setTargetVelocity(profileState.getV());
        feedforward.setTargetAcceleration(profileState.getA());

        double fullStateOutput = fullState.update(pCurrent, vCurrent, aCurrent);

        return fullStateOutput + feedforward.update(voltage, fullStateOutput);
    }

    public double update(double pCurrent, double vCurrent, double aCurrent) {
        return this.update(pCurrent, vCurrent, aCurrent, 12.0);
    }

    public double update(double pCurrent, double vCurrent) {
        return this.update(pCurrent, vCurrent, 0.0);
    }

    public double update(double pCurrent) {
        return this.update(pCurrent, 0.0);
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
        return profileTimer.seconds() >= profile.duration();
    }
}
