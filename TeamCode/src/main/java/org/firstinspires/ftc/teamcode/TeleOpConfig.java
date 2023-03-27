package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {
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
    public static double LIFT_kG_FOUR = 0.2;
    public static double LIFT_kG_THREE = 0.12;
    public static double LIFT_kG_TWO = 0.08;
    public static double LIFT_kG_ONE = 0.03;

    public static double LIFT_kP = 0.0;
    public static double LIFT_kI = 0.0;
    public static double LIFT_kD = 0.0;
    public static double LIFT_kV = 0.028;
    public static double LIFT_kA = 0.004;
    public static double LIFT_kS = 0.007;

    public static double LIFT_VELO_FILTER_GAIN = 0.796;

    public static double LIFT_MAX_VELO = 36; // inches per second
    public static double LIFT_MAX_ACCEL = 48; // inches per second^2
    public static double LIFT_MAX_JERK = 120; // inches per second^3

    public static double LIFT_INTEGRATION_MAX_VELO = 0.5;
    public static double LIFT_PID_FILTER_GAIN = 0.8;
    public static double LIFT_E_TOLERANCE = 0.15843625; // inches

    public static double LIFT_TICKS_PER_INCH = 0.03168725;

    public static double LIFT_RESET_VELOCITY = -0.2;
    public static double PRECISION_MODE_SCALE = 0.3;

    public static double CLAW_CLOSED_ANGLE = 165;
    public static double CLAW_PASS_ANGLE = 140;
    public static double CLAW_OPEN_ANGLE = 115;

    public static double CLAW_CLOSING_TIME = 0.3;

    public static double PIVOT_FRONT_ANGLE = 198;
    public static double PIVOT_BACK_ANGLE = 2;

    public static double PASS_RIGHT_FRONT_ANGLE = 5;
    public static double PASS_RIGHT_BACK_ANGLE = 197;

    public static double PASS_LEFT_FRONT_ANGLE = 225;
    public static double PASS_LEFT_BACK_ANGLE = 25;

    public static double PASS_RIGHT_PIVOT_ANGLE = 45;
    public static double PASS_LEFT_PIVOT_ANGLE = 185;

    public static double FRONT_TO_PIVOT_TIME = 0.1; // time between lifting claw and pivoting
    public static double PIVOTING_TO_BACK_TIME = 0.65; // time between starting pivoting and moving to back
    public static double PIVOT_TO_BACK_TIME = 0.2; // time between starting to move to the back and reaching the back

    public static double BACK_TO_PIVOT_TIME = 0.55; //time between moving from back to pivot position
    public static double PIVOTING_TO_FRONT_TIME = 0.45; // time from starting pivoting to moving down
    public static double PIVOT_TO_FRONT_TIME = 0.1; // time it takes to get from pivot position to front position

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
}
