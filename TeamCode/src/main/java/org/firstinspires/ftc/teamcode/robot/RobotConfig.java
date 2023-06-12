package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    public static double
            HEIGHT_FLOOR = 0,
            HEIGHT_2 = 1.35,
            HEIGHT_LOW = 6.2,
            HEIGHT_MEDIUM = 16.4,
            HEIGHT_TALL = 26.6,
            HEIGHT_STAGES_4 = 28.8,
            HEIGHT_STAGES_3 = 19.2,
            HEIGHT_STAGES_2 = 9.6,
            LIFT_kG_4 = 0.18,
            LIFT_kG_3 = 0.14,
            LIFT_kG_2 = 0.1,
            LIFT_kG_1 = 0.06,
            LIFT_kP = 0.15,
            LIFT_kI = 0.21,
            LIFT_kD = 0.0,
            LIFT_UP_kV = 0.019,
            LIFT_UP_kA = 0.001,
            LIFT_DOWN_kV = 0.0087,
            LIFT_DOWN_kA = 0.0005,
            LIFT_kS = 0.0,
            LIFT_kD_FILTER_GAIN = 0.98,
            LIFT_VELO_FILTER_GAIN = 0.8,
            LIFT_ACCEL_FILTER_GAIN = 0.8,
            LIFT_MAX_VELO = 39,
            LIFT_MAX_ACCEL = 200,
            LIFT_MAX_JERK = 600,
            LIFT_INTEGRATION_MAX_VELO = 0.6,
            LIFT_POS_TOLERANCE = 0.15843625,
            LIFT_INCHES_PER_TICK = 0.03168725,
            PRECISION_MODE_SCALE = 0.3,
            ANGLE_CLAW_CLOSED = 0,
            ANGLE_CLAW_OPEN = 62,
            ANGLE_ARM_DOWN = 100,
            ANGLE_PIVOT_FRONT = 17,
            ANGLE_PIVOT_BACK = 216,
            ANGLE_PASS_FRONT = 8,
            ANGLE_PASS_TILT = 45,
            ANGLE_PASS_BACK = 310,
            ANGLE_PASS_MINI_TILT = 17,
            TIME_CLAW = 0.0,
            TIME_LIFT_MEDIUM = 0.8,
            TIME_LIFT_TALL = 1,
            PASS_PIVOT_POS = (ANGLE_PASS_BACK - ANGLE_PASS_FRONT) * 0.5,
            PASS_PIVOT_POS_TOLERANCE = 30,
            PASS_MAX_VELO = 600,
            PASS_MAX_ACCEL = 3000,
            PASS_MAX_JERK = 7000;
    public static int
            LIFT_kD_FILTER_COUNT = 5,
            LIFT_VELO_FILTER_COUNT = 5,
            LIFT_ACCEL_FILTER_COUNT = 5;
}
