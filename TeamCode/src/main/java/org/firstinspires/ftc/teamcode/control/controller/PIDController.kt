package org.firstinspires.ftc.teamcode.control.controller

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter
import kotlin.math.sign

/**
 * PID controller
 */
class PIDController
/**
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param filterGain derivative filter smoothness, increases over the interval 0 <= x < 1
 * @param clock clock
 */
@JvmOverloads constructor(
    private var kP: Double,
    private var kI: Double,
    private var kD: Double,
    private var filterGain: Double = 0.8,
    private val clock: ElapsedTime = ElapsedTime()
) {
    var lastError: Double = 0.0
    var errorSum: Double = 0.0
    var errorDeriv: Double = 0.0
    var integrate: Boolean = true

    private var derivFilter: IIRLowPassFilter = IIRLowPassFilter(filterGain)

    fun setGains(
        kP: Double,
        kI: Double,
        kD: Double,
        filterGain: Double = this.filterGain
    ) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.filterGain = filterGain
        derivFilter.setGain(filterGain)
    }

    /**
     * Target position (the controller setpoint).
     */
    var targetPosition: Double = 0.0

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position
     */
    fun update(measuredPosition: Double): Double {
        val error = targetPosition - measuredPosition
        val dt = clock.seconds()
        return if (dt == 0.0) {
            lastError = error
            0.0
        } else {
            errorDeriv = derivFilter.getEstimate((error - lastError) / dt)

            if (integrate) errorSum += 0.5 * (error + lastError) * dt
            if (sign(error) != sign(lastError)) resetIntegral()

            lastError = error
            clock.reset()

            (kP * error) + (kI * errorSum) + (kD * errorDeriv)
        }
    }

    /**
     * Reset the controller's integral sum.
     */
    fun resetIntegral() {
        errorSum = 0.0
        lastError = 0.0
        integrate = true
    }
}
