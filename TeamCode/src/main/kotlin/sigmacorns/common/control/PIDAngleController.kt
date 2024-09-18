package sigmacorns.common.control

import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians

open class PIDAngleController(var coefficients: PIDCoefficients) {

    constructor(p: Number, i: Number, d: Number) : this(PIDCoefficients(p.toDouble(), i.toDouble(), d.toDouble()))

    var target = 0.0
        set(value) { field = normalizeRadians(value) }
    var position = 0.0
        set(value) { field = normalizeRadians(value) }

    /**
     * The accumulated integral
     */
    var integral = 0.0

    /**
     * The error from the previous loop
     */
    var previousError = 0.0

    fun reset() {
        integral = 0.0
        target = 0.0
        previousError = 0.0
    }

    fun update(deltaTime: Double): Double {
        val error = normalizeRadians(target - position)
        integral += error * deltaTime
        var derivative = (error - previousError) / deltaTime
        if (derivative.isNaN()) derivative = 0.0
        previousError = error

        return (coefficients.p * error) + (coefficients.i * integral) + (coefficients.d * derivative)
    }

    fun copy(): PIDAngleController {
        return PIDAngleController(coefficients)
    }
}