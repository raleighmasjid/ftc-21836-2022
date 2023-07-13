package org.firstinspires.ftc.teamcode.control.controllers.pid;

import org.firstinspires.ftc.teamcode.control.MotionProfiler;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;

public class ProfiledPIDVA extends PIDVAController {

    private final MotionProfiler profiler = new MotionProfiler();

    /**
     * Initialize fields <p>
     *
     * @param pid         PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public ProfiledPIDVA(PIDController pid, FeedforwardController feedforward) {
        super(pid, feedforward);
    }

    public ProfiledPIDVA() {
        this(new PIDController(), new FeedforwardController());
    }

    public void updateConstraints(
            double MAX_VELO,
            double MAX_ACCEL,
            double MAX_JERK
    ) {
        profiler.updateConstraints(
                MAX_VELO,
                MAX_ACCEL,
                MAX_JERK
        );
    }

    @Override
    public void setTarget(State target) {
        setTarget(new State(0.0, 0.0), target);
    }

    public void setTarget(State current, State target) {
        profiler.generateProfile(current, target);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     * @param voltage     measured battery voltage (for feedforward voltage correction)
     */
    @Override
    public double calculate(State measurement, double voltage) {
        profiler.update();
        super.setTarget(new State(profiler.getX(), profiler.getV(), profiler.getA()));
        return super.calculate(measurement, voltage);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    @Override
    public double calculate(State measurement) {
        return this.calculate(measurement, 12.0);
    }

    public double getX() {
        return profiler.getX();
    }

    public double getV() {
        return profiler.getV();
    }

    public double getA() {
        return profiler.getA();
    }
}
