package org.firstinspires.ftc.teamcode.robot;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    // inches
    public static double
            HEIGHT_FLOOR = 0,
            HEIGHT_TWO = 1.25,
            HEIGHT_THREE = 2.5,
            HEIGHT_FOUR = 3.75,
            HEIGHT_FIVE = 5,

    HEIGHT_LOW = 15.8365,
            HEIGHT_MEDIUM = 25.365,
            HEIGHT_TALL = 36.50375,

    STAGES_FOUR = 28.8,
            STAGES_THREE = 19.2,
            STAGES_TWO = 9.6,

    LIFT_kG_FOUR = 0.075,
            LIFT_kG_THREE = 0.055,
            LIFT_kG_TWO = 0.025,
            LIFT_kG_ONE = 0.0,

    LIFT_kP = 0.0,
            LIFT_kI = 0.0,
            LIFT_kD = 0.0,
            LIFT_kV = 0.0,
            LIFT_kA = 0.0,
            LIFT_kS = 0.0,

    LIFT_PID_FILTER_GAIN = 0.98,
            LIFT_VELO_FILTER_GAIN = 0.796,
            LIFT_ACCEL_FILTER_GAIN = 0.95,
            LIFT_JERK_FILTER_GAIN = 0.95,

    LIFT_MAX_UP_VELO = 10,
            LIFT_MAX_UP_ACCEL = 10,
            LIFT_MAX_DOWN_VELO = 10,
            LIFT_MAX_DOWN_ACCEL = 10,
            LIFT_MAX_JERK = 10,

    LIFT_INTEGRATION_MAX_VELO = 0.8,
            LIFT_POS_TOLERANCE = 0.15843625,
            LIFT_TICKS_PER_INCH = 0.03168725,

    PRECISION_MODE_SCALE = 0.3,

    CLAW_CLOSED_ANGLE = 165,
            CLAW_PASS_ANGLE = 140,
            CLAW_OPEN_ANGLE = 115,

    ARM_L_DOWN_ANGLE = 0,
            ARM_L_UP_ANGLE = 0,
            ARM_R_DOWN_ANGLE = 0,
            ARM_R_UP_ANGLE = 0,

    PIVOT_FRONT_ANGLE = 198,
            PIVOT_BACK_ANGLE = 2,

    PASS_FRONT_ANGLE = 5,
            PASS_FRONT_TILT_ANGLE = 5,
            PASS_PIVOT_ANGLE = 45,
            PASS_BACK_ANGLE = 197,
            PASS_BACK_TILT_ANGLE = 197,

    TIME_CLAW = 0.3,
            TIME_FRONT_PIVOT = 0.1,
            TIME_PIVOTING = 0.65,
            TIME_BACK_PIVOT = 0.2,

    camera_fx = 578.272,
            camera_fy = 578.272,
            camera_cx = 402.145,
            camera_cy = 221.506;

    public static int
            LIFT_PID_ESTIMATE_COUNT = 5,
            LIFT_VELO_ESTIMATE_COUNT = 5,
            LIFT_ACCEL_ESTIMATE_COUNT = 5,
            LIFT_JERK_ESTIMATE_COUNT = 5;
}
