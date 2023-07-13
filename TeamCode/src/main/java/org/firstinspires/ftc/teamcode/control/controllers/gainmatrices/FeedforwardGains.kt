package org.firstinspires.ftc.teamcode.control.controllers.gainmatrices

data class FeedforwardGains @JvmOverloads constructor(
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kStatic: Double = 0.0
)
