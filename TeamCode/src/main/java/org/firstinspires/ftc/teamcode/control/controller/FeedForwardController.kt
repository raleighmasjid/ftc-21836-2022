package org.firstinspires.ftc.teamcode.control.controller

import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.sign

/**
 * FeedForward controller
 */
class FeedforwardController
/**
 * Constructor for [FeedforwardController]. [kV], [kA], and [kStatic] are designed for DC motor feedforward
 * control (the most common kind of feedforward in FTC).
 *
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic additive feedforward constant
 * @param clock clock
 */
@JvmOverloads constructor(
    private var kV: Double = 0.0,
    private var kA: Double = 0.0,
    private var kStatic: Double = 0.0,
    private val clock: NanoClock = NanoClock.system()
) {

    fun setGains(
        kV: Double = this.kV,
        kA: Double = this.kA,
        kStatic: Double = this.kStatic
    ) {
        this.kV = kV
        this.kA = kA
        this.kStatic = kStatic
    }

    /**
     * Target velocity.
     */
    var targetVelocity: Double = 0.0

    /**
     * Target acceleration.
     */
    var targetAcceleration: Double = 0.0

    /**
     * Run a single iteration of the controller.
     */
    fun update(): Double {
        val baseOutput = kV * targetVelocity + kA * targetAcceleration
        return if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput + sign(baseOutput) * kStatic
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param additionalOutput output from another joint controller (like a PID) that you want to add to the calculation of the sign of kS
     */
    fun update(additionalOutput: Double): Double {
        val baseOutput = kV * targetVelocity + kA * targetAcceleration
        return (if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput) + (sign(baseOutput + additionalOutput) * kStatic)
    }
}
