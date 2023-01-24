package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {
    public static double HEIGHT_ONE = 0;
    public static double HEIGHT_TWO = 0;
    public static double HEIGHT_THREE = 0;
    public static double HEIGHT_FOUR = 0;
    public static double HEIGHT_FIVE = 0;

    public static double HEIGHT_GROUND = 5;
    public static double HEIGHT_LOW = 500;
    public static double HEIGHT_MEDIUM = 800;
    public static double HEIGHT_TALL = 1152;

    public static double MINIMUM_PIVOT_HEIGHT = 100;

    public static double LIFT_P = 0.01;
    public static double LIFT_I = 0;
    public static double LIFT_D = 0;
    public static double LIFT_F = 0.00013;

    public static double LIFT_E_TOLERANCE = 1;
    public static double LIFT_V_TOLERANCE = 10;

    public static double LIFT_MAX_DOWN_VELOCITY = -0.35;

    public static double CLAW_RIGHT_CLOSED = 165;
    public static double CLAW_RIGHT_OPEN = 115;
    public static double CLAW_RIGHT_PASS = 140;

    public static double CLAW_CLOSING_TIME = 0.31;
    public static double CLAW_PASS_CLOSING_TIME = 0.155;
    public static double CLAW_DROP_TIME = 0.7;
    public static double CLAW_LIFT_TIME = 0.7;
    public static double PASSTHROUGH_TIME = 1.2;

    public static double PIVOT_FRONT = 200;
    public static double PIVOT_BACK = 0;

    public static double PASS_1_FRONT = 5;
    public static double PASS_1_BACK = 195;

    public static double PASS_2_FRONT = 195;
    public static double PASS_2_BACK = 5;

    public static double PASS_1_PIVOTING = 25;
    public static double PASS_2_PIVOTING = 170;

    public static double FRONT_TO_PIVOT_TIME = 0.1;
    public static double PIVOTING_TO_BACK_TIME = 0.75;
    public static double PIVOT_TO_BACK_TIME = 0.4;

    public static double BACK_TO_PIVOT_TIME = 0.857;
    public static double PIVOTING_TO_FRONT_TIME = 0.6;
    public static double PIVOT_TO_FRONT_TIME = 0.1;

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
}