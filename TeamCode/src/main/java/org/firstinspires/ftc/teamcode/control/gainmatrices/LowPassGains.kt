package org.firstinspires.ftc.teamcode.control.gainmatrices

data class LowPassGains @JvmOverloads constructor(
    @JvmField var gain: Double = 0.5,
    @JvmField var count: Int = 10,
)
