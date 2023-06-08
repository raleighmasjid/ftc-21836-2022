package org.firstinspires.ftc.teamcode.control.controller

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
 */
@JvmOverloads constructor(
    private var kV: Double = 0.0,
    private var kA: Double = 0.0,
    private var kStatic: Double = 0.0,
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
    var targetVelocity = 0.0

    /**
     * Target acceleration.
     */
    var targetAcceleration = 0.0

    /**
     * Run a single iteration of the controller.
     */
    fun update(): Double {
        return update(12.0, 0.0)
    }

    /**
     * Run a single iteration of the controller.
     */
    fun update(voltage: Double): Double {
        return update(voltage, 0.0)
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param additionalOutput output from a joint controller (like a PID) that is included in the calculation of the sign of kS
     */
    fun update(voltage: Double additionalOutput: Double): Double {
        val baseOutput = kV * targetVelocity + kA * targetAcceleration
        return ((if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput) + (sign(baseOutput + additionalOutput) * kStatic)) * (12.0 / voltage)
    }
}
