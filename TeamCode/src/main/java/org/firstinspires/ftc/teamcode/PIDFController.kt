package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/**
 * PID controller with various feedforward components.
 */
class PIDFController
/**
 * Constructor for [PIDFController]. [kV], [kA], and [kStatic] are designed for DC motor feedforward
 * control (the most common kind of feedforward in FTC). [kF] provides a custom feedforward term for other plants.
 *
 * @param pid traditional PID coefficients
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic additive feedforward constant
 * @param kF custom feedforward that depends on position and/or velocity (e.g., a gravity term for arms)
 * @param clock clock
 */
@JvmOverloads constructor(
    private var pid: PIDCoefficients,
    private var kV: Double = 0.0,
    private var kA: Double = 0.0,
    private var kStatic: Double = 0.0,
    private var kF: (Double, Double?) -> Double = { _, _ -> 0.0 },
    private val clock: NanoClock = NanoClock.system()
) {
    lateinit var coefficients: PIDCoefficients
    private var errorSum: Double = 0.0
    private var lastUpdateTimestamp: Double = Double.NaN

    private var inputBounded: Boolean = false
    private var minInput: Double = 0.0
    private var maxInput: Double = 0.0

    private var outputBounded: Boolean = false
    private var minOutput: Double = 0.0
    private var maxOutput: Double = 0.0
    private var a = TeleOpConfig.LIFT_FILTER_GAIN // a can be anything from 0 < a < 1
    private var lastFilterEstimate = 0.0
    private var currentFilterEstimate = 0.0
    private var integralMax = if(TeleOpConfig.LIFT_kI == 0.0) {
        10.0
    } else {
        1.0/TeleOpConfig.LIFT_kI
    }
    private var lastTargetPosition = 0.0

    fun setCoefficients(
        pid: PIDCoefficients,
        kV: Double = 0.0,
        kA: Double = 0.0,
        kStatic: Double = 0.0
    ) {
        this.pid = pid
        this.kV = kV
        this.kA = kA
        this.kStatic = kStatic
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
    var lastError: Double = 0.0
        private set

    /**
     * Sets bound on the input of the controller. The min and max values are considered modularly-equivalent (that is,
     * the input wraps around).
     *
     * @param min minimum input
     * @param max maximum input
     */
    fun setInputBounds(min: Double, max: Double) {
        if (min < max) {
            inputBounded = true
            minInput = min
            maxInput = max
        }
    }

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
        return if (lastUpdateTimestamp.isNaN()) {
            lastError = error
            lastUpdateTimestamp = currentTimestamp
            lastFilterEstimate = currentFilterEstimate
            lastTargetPosition = targetPosition
            0.0
        } else {
            val dt = currentTimestamp - lastUpdateTimestamp
            currentFilterEstimate = (a * lastFilterEstimate) + ((1-a) * (error - lastError))
            val errorDeriv = currentFilterEstimate / dt

            if (errorSum > integralMax) {
                errorSum = integralMax
            }
            if (errorSum < -integralMax){
                errorSum = -integralMax
            }
            if (-errorDeriv <= TeleOpConfig.LIFT_INTEGRAL_MIN_VELO && errorDeriv >= -TeleOpConfig.LIFT_INTEGRAL_MIN_VELO) {
                errorSum += 0.5 * (error + lastError) * dt
            }
            if (sign(lastTargetPosition) != sign(targetPosition)) {
                reset()
            }

            lastError = error
            lastUpdateTimestamp = currentTimestamp
            lastFilterEstimate = currentFilterEstimate
            lastTargetPosition = targetPosition

            // note: we'd like to refactor this with Kinematics.calculateMotorFeedforward() but kF complicates the
            // determination of the sign of kStatic
            val baseOutput = pid.kP * error + pid.kI * errorSum +
                pid.kD * (measuredVelocity?.let { targetVelocity - it } ?: errorDeriv) +
                kV * targetVelocity + kA * targetAcceleration + kF(measuredPosition, measuredVelocity)
            val output = if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput + sign(baseOutput) * kStatic

            if (outputBounded) {
                max(minOutput, min(output, maxOutput))
            } else {
                output
            }
        }
    }

    /**
     * Reset the controller's integral sum.
     */
    fun reset() {
        errorSum = 0.0
        lastError = 0.0
        lastUpdateTimestamp = Double.NaN
    }
}
