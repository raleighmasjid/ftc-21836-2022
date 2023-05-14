package org.firstinspires.ftc.teamcode.control.controller

import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/**
 * PID controller with various feedforward components.
 */
class FeedForwardController
/**
 * Constructor for [FeedForwardController]. [kV], [kA], and [kStatic] are designed for DC motor feedforward
 * control (the most common kind of feedforward in FTC). [kF] provides a custom feedforward term for other plants.
 *
 * @param pid traditional PID coefficients
 * @param maxIntegrationVelocity max velocity that integral path will continue integration
 * @param filterGain derivative filter weight, 0 = unsmoothed, 0<x<1 increasingly smoothed, 1 = broken
 * @param estimateCount number of past values to average when filtering
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic additive feedforward constant
 * @param kF custom feedforward that depends on position and/or velocity (e.g., a gravity term for arms)
 * @param clock clock
 */
@JvmOverloads constructor(
    private var kP: Double,
    private var kI: Double,
    private var kD: Double,
    private var maxIntegrationVelocity: Double = 0.5,
    private var filterGain: Double = 0.8,
    private var estimateCount: Int = 5,
    private var kV: Double = 0.0,
    private var kA: Double = 0.0,
    private var kStatic: Double = 0.0,
    private var kF: (Double, Double?) -> Double = { _, _ -> 0.0 },
    private val clock: NanoClock = NanoClock.system()
) {
    private var errorSum: Double = 0.0
    private var lastUpdateTimestamp: Double = Double.NaN

    private var inputBounded: Boolean = false
    private var minInput: Double = 0.0
    private var maxInput: Double = 0.0

    private var outputBounded: Boolean = false
    private var minOutput: Double = 0.0
    private var maxOutput: Double = 0.0

    var currentFilterEstimate: Double = 0.0
    private var integrate: Boolean = true
    var positionTolerance: Double = 2.0

    private var derivFilter: IIRLowPassFilter =
        IIRLowPassFilter(
            filterGain
        )

    fun setGains(
        kP: Double,
        kI: Double,
        kD: Double,
        maxIntegrationVelocity: Double = this.maxIntegrationVelocity,
        filterGain: Double = this.filterGain,
        estimateCount: Int = this.estimateCount,
        kV: Double = this.kV,
        kA: Double = this.kA,
        kStatic: Double = this.kStatic
    ) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.maxIntegrationVelocity = maxIntegrationVelocity
        this.filterGain = filterGain
        this.estimateCount = estimateCount
        this.kV = kV
        this.kA = kA
        this.kStatic = kStatic
        derivFilter.setGain(filterGain)
    }

    /**
     * Target position (that is, the controller setpoint).
     */
    var targetPosition: Double = 0.0

    /**
     * Target velocity.
     */
    var targetVelocity: Double = 0.0

    /**
     * Target acceleration.
     */
    var targetAcceleration: Double = 0.0

    /**
     * Error computed in the last call to [update].
     */
    private var lastError: Double = 0.0

//    /**
//     * Sets bound on the input of the controller. The min and max values are considered modularly-equivalent (that is,
//     * the input wraps around).
//     *
//     * @param min minimum input
//     * @param max maximum input
//     */
//    fun setInputBounds(min: Double, max: Double) {
//        if (min < max) {
//            inputBounded = true
//            minInput = min
//            maxInput = max
//        }
//    }

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    fun setOutputBounds(min: Double, max: Double) {
        if (min < max) {
            outputBounded = true
            minOutput = min
            maxOutput = max
            maxIntegrationVelocity = min(maxOutput, maxIntegrationVelocity)
        }
    }

    private fun getPositionError(measuredPosition: Double): Double {
        var error = targetPosition - measuredPosition
        if (inputBounded) {
            val inputRange = maxInput - minInput
            while (abs(error) > inputRange / 2.0) {
                error -= sign(error) * inputRange
            }
        }
        return error
    }

    fun atTargetPosition(measuredPosition: Double): Boolean {
        return abs(getPositionError(measuredPosition)) <= positionTolerance
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position (feedback)
     * @param measuredVelocity measured velocity
     */
    @JvmOverloads
    fun update(
        measuredPosition: Double,
        measuredVelocity: Double? = null
    ): Double {
        val currentTimestamp = clock.seconds()
        val error = getPositionError(measuredPosition)
        val dy = error - lastError
        currentFilterEstimate = derivFilter.getEstimate(dy)
        return if (lastUpdateTimestamp.isNaN()) {
            lastError = error
            lastUpdateTimestamp = currentTimestamp
            0.0
        } else {
            val dt = currentTimestamp - lastUpdateTimestamp
            val errorDeriv = currentFilterEstimate / dt

            errorSum += if (integrate) (0.5 * (error + lastError) * dt) else 0.0

            if (sign(error) != sign(lastError)) reset()

            lastError = error
            lastUpdateTimestamp = currentTimestamp

            // note: we'd like to refactor this with Kinematics.calculateMotorFeedforward() but kF complicates the
            // determination of the sign of kStatic
            val baseOutput = kP * error +
                    kI * errorSum +
                    kD * (measuredVelocity?.let { targetVelocity - it } ?: errorDeriv) +
                    kV * targetVelocity +
                    kA * targetAcceleration +
                    kF(
                        measuredPosition,
                        measuredVelocity
                    )
            val output =
                if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput + sign(baseOutput) * kStatic

            integrate = !(abs(output) >= maxIntegrationVelocity && sign(output) == sign(error))

            if (outputBounded) max(minOutput, min(output, maxOutput)) else output
        }
    }

    /**
     * Reset the controller's integral sum.
     */
    fun reset() {
        errorSum = 0.0
        lastError = 0.0
        integrate = true
    }
}
