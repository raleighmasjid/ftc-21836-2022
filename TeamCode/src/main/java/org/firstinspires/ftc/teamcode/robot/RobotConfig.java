package org.firstinspires.ftc.teamcode.robot;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    // inches
    public static double HEIGHT_FLOOR = 0;
    public static double HEIGHT_TWO = 1.25;
    public static double HEIGHT_THREE = 2.5;
    public static double HEIGHT_FOUR = 3.75;
    public static double HEIGHT_FIVE = 5;
    // inches
    public static double HEIGHT_LOW = 15.8365;
    public static double HEIGHT_MEDIUM = 25.365;
    public static double HEIGHT_TALL = 36.50375;
    // inches
    public static double STAGES_FOUR = 28.8;
    public static double STAGES_THREE = 19.2;
    public static double STAGES_TWO = 9.6;
    public static double MINIMUM_PIVOT_HEIGHT = 3.325;
    // inches per second
    public static double LIFT_kG_FOUR = 0.075;
    public static double LIFT_kG_THREE = 0.055;
    public static double LIFT_kG_TWO = 0.025;
    public static double LIFT_kG_ONE = 0.0;

    public static double LIFT_kP = 0.0;
    public static double LIFT_kI = 0.0;
    public static double LIFT_kD = 0.0;
    public static double LIFT_kV = 0.0;
    public static double LIFT_kA = 0.0;
    public static double LIFT_kS = 0.0;

    public static double LIFT_VELO_FILTER_GAIN = 0.796;
    public static double LIFT_ACCEL_FILTER_GAIN = 0.95;
    public static double LIFT_JERK_FILTER_GAIN = 0.95;

    public static int LIFT_VELO_ESTIMATE_COUNT = 5;
    public static int LIFT_ACCEL_ESTIMATE_COUNT = 5;
    public static int LIFT_JERK_ESTIMATE_COUNT = 5;

    public static double LIFT_MAX_UP_VELO = 10; // inches per second
    public static double LIFT_MAX_UP_ACCEL = 10; // inches per second^2
    
    public static double LIFT_MAX_DOWN_VELO = 10; // inches per second
    public static double LIFT_MAX_DOWN_ACCEL = 10; // inches per second^2
    
    public static double LIFT_MAX_JERK = 10; // inches per second^3

    public static double LIFT_INTEGRATION_MAX_VELO = 0.8;
    public static double LIFT_PID_FILTER_GAIN = 0.98;
    public static int LIFT_PID_ESTIMATE_COUNT = 5;
    public static double LIFT_POS_TOLERANCE = 0.15843625; // inches

    public static double LIFT_TICKS_PER_INCH = 0.03168725;

    public static double PRECISION_MODE_SCALE = 0.3;

    public static double CLAW_CLOSED_ANGLE = 165;
    public static double CLAW_PASS_ANGLE = 140;
    public static double CLAW_OPEN_ANGLE = 115;

    public static double TIME_CLAW = 0.3;
    public static double TIME_CLAW_DROP = 0.0;

    public static double PIVOT_FRONT_ANGLE = 198;
    public static double PIVOT_BACK_ANGLE = 2;

    public static double PASS_FRONT_ANGLE = 5;
    public static double PASS_FRONT_TILT_ANGLE = 5;
    public static double PASS_PIVOT_ANGLE = 45;
    public static double PASS_BACK_ANGLE = 197;
    public static double PASS_BACK_TILT_ANGLE = 197;

    public static double TIME_FRONT_PIVOT = 0.1;
    public static double TIME_PIVOTING = 0.65;
    public static double TIME_BACK_PIVOT = 0.2;

    public static double camera_fx = 578.272;
    public static double camera_fy = 578.272;
    public static double camera_cx = 402.145;
    public static double camera_cy = 221.506;
}
