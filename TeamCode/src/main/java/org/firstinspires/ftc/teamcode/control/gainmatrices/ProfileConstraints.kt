package org.firstinspires.ftc.teamcode.control.gainmatrices

data class ProfileConstraints @JvmOverloads constructor(
    @JvmField var MAX_VELO: Double = 1.0,
    @JvmField var MAX_ACCEL: Double = 1.0,
    @JvmField var MAX_JERK: Double = 0.0,
)
