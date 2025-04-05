package sigmacorns.common.control

import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.mapRanges
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.revolutions
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign


data class ServoControlLoopState(val estimatedPos: Expression, val estimatedVel: Expression)

class SimpleServoController(
    val bounds: Bounds<Radian>,
    val maxSpeed: Expression, // rad/s
    val tolerance: Radian,
    override var position: ServoControlLoopState,
): Controller<ServoControlLoopState, Double, Radian>() {
    private var posMap = mapRanges(bounds.let { it.min.value..it.max.value },0.0..1.0)

    override var target: Radian = position.estimatedPos.cast(rad)

    override fun copy() = SimpleServoController(bounds,maxSpeed,tolerance,position)

    override var output: Double = update(0.0)

    override fun update(dt: Double): Double {
        val targetPos = bounds.apply(target)

        val diff = targetPos-position.estimatedPos
        val dx = diff.map { min(it.absoluteValue,dt*maxSpeed.value)*it.sign }

        position = ServoControlLoopState(bounds.apply((position.estimatedPos+dx).cast(rad)),0.rad/s)

        var theta: Expression = target
        if(theta.value.isNaN()) {
            println("WARNING: SERVO POSITION IS NaN")
            return Double.NaN
        }
        while (theta<bounds.min) theta += 1.revolutions
        while (theta>bounds.max) theta -= 1.revolutions

        //cannot reach, choose closest bound
        if(theta<bounds.min) {
            println("WARNING: SERVO TARGET IS OUTSIDE OF BOUNDS")
            val minDist = (target-bounds.min).normalizeRadian().map { it.absoluteValue }
            val maxDist = (target-bounds.max).normalizeRadian().map { it.absoluteValue }
            theta = if(minDist < maxDist) bounds.min else bounds.max
        }

        val servoPos = (posMap)(theta.value).toDouble()

        return servoPos
    }

    fun reached()
            = (position.estimatedPos-bounds.apply(target)).normalizeRadian().map { it.absoluteValue } < tolerance
}
