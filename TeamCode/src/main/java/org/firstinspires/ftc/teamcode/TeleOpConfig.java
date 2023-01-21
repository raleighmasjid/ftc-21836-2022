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

    public static double LIFT_P = 0.01;
    public static double LIFT_I = 0;
    public static double LIFT_D = 0;
    public static double LIFT_F = 0.00013;

    public static double LIFT_E_TOLERANCE = 1;
    public static double LIFT_V_TOLERANCE = 10;

    public static double CLAW_RIGHT_CLOSED = 165;
    public static double CLAW_RIGHT_OPEN = 115;

    public static double pivotFront = 200;
    public static double pivotBack = 0;

    public static double pass1Front = 5;
    public static double pass1Back = 195;

    public static double pass2Front = 195;
    public static double pass2Back = 5;

    public static double pass1Pivoting = 25;
    public static double pass2Pivoting = 170;

    public static double clawClosingTime = 0.31;
    public static double frontToPivot = 0.2;
    public static double backToPivot = 0.857;
    public static double pivotingTime = 0.85;

    public static double liftDownMaxVelo = -0.3;
}