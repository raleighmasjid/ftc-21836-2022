package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {
    public static double HEIGHT_ONE = 0;
    public static double HEIGHT_TWO = 40;
    public static double HEIGHT_THREE = 80;
    public static double HEIGHT_FOUR = 120;
    public static double HEIGHT_FIVE = 150;

    public static double HEIGHT_GROUND = 30;
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

    public static double LIFT_RESET_VELOCITY = -0.2;
    public static double LIFT_RESET_TIMER = 0.2;
    public static double LIFT_MAX_DOWN_VELOCITY = -0.45;
    public static double LIFT_MANUAL_CONTROL_SCALE = 15;
    public static double PRECISION_MODE_SCALE = 0.3;

    public static double CLAW_CLOSED_ANGLE = 165;
    public static double CLAW_PASS_ANGLE = 140;
    public static double CLAW_OPEN_ANGLE = 115;

    public static double CLAW_CLOSING_TIME = 0.3;
    public static double DROP_TO_RETRACT_TIME = 0.15;

    public static double AUTON_START_DELAY = 0.16;

    public static double PIVOT_FRONT_ANGLE = 198;
    public static double PIVOT_BACK_ANGLE = 2;

    public static double PASS_RIGHT_FRONT_ANGLE = 5;
    public static double PASS_RIGHT_BACK_ANGLE = 197;
    public static double PASS_RIGHT_FRONT_UP_ANGLE = 101;
    public static double PASS_RIGHT_BACK_UP_ANGLE = 101;

    public static double PASS_LEFT_FRONT_ANGLE = 225;
    public static double PASS_LEFT_BACK_ANGLE = 25;
    public static double PASS_LEFT_FRONT_UP_ANGLE = 125;
    public static double PASS_LEFT_BACK_UP_ANGLE = 125;

    public static double PASS_RIGHT_PIVOT_ANGLE = 45;
    public static double PASS_LEFT_PIVOT_ANGLE = 185;

    public static double FRONT_TO_PIVOT_TIME = 0.1; // time between lifting claw and pivoting
    public static double PIVOTING_TO_BACK_TIME = 0.8; // time between starting pivoting and moving to back
    public static double PIVOT_TO_BACK_TIME = 0.2; // time between starting to move to the back and reaching the back

    public static double BACK_TO_PIVOT_TIME = 0.68; //time between moving from back to pivot position
    public static double PIVOTING_TO_FRONT_TIME = 0.6; // time from starting pivoting to moving down
    public static double PIVOT_TO_FRONT_TIME = 0.1; // time it takes to get from pivot position to front position

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
}
