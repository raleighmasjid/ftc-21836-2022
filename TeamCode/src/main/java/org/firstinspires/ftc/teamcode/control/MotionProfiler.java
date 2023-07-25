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

    private ProfileConstraints constraints = new ProfileConstraints();

    public void updateConstraints(ProfileConstraints constraints) {
        this.constraints = constraints;
    }

    public void generateProfile(State current, State target) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(current.x, current.v),
                new MotionState(target.x, target.v),
                constraints.maxV,
                constraints.maxA,
                constraints.maxJ
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
