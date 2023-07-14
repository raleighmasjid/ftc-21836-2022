package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.ProfileConstraints;

public class MotionProfiler {

    private MotionProfile profile;

    private MotionState profileState = new MotionState(0.0, 0.0, 0.0, 0.0);

    private final ElapsedTime profileTimer = new ElapsedTime();

    private double MAX_VELO = 1.0;
    private double MAX_ACCEL = 1.0;
    private double MAX_JERK = 0.0;

    public void updateConstraints(ProfileConstraints constraints) {
        this.MAX_VELO = constraints.MAX_VELO;
        this.MAX_ACCEL = constraints.MAX_ACCEL;
        this.MAX_JERK = constraints.MAX_JERK;
    }

    public void generateProfile(State current, State target) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(current.getX(), current.getV()),
                new MotionState(target.getX(), target.getV()),
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );
        profileTimer.reset();
    }

    public void update() {
        profileState = profile.get(profileTimer.seconds());
    }

    public double getX() {
        return profileState.getX();
    }

    public double getV() {
        return profileState.getV();
    }

    public double getA() {
        return profileState.getA();
    }

    public double getJ() {
        return profileState.getJ();
    }

    public boolean isDone() {
        return profileTimer.seconds() >= profile.duration();
    }
}
