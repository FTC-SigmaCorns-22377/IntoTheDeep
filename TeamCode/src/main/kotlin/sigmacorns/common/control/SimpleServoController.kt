package sigmacorns.common.control

import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math.clamp
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.mapRanges
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.revolutions
import sigmacorns.common.io.SigmaIO
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt


// TODO: change to estimator.
class SimpleServoController(
    val servo: Actuator<Double>,
    val bounds: Bounds<Radian>,
    val maxSpeed: Expression, // rad/s
    val tolerance: Radian,
    override var x: ServoControlLoopState,
    io: SigmaIO
): ControlLoop<ServoControlLoopState, Double, Radian>("servo",io) {
    private var posMap = mapRanges(bounds.let { it.min.value..it.max.value },0.0..1.0)

    override var t: Radian = x.estimatedPos.cast(rad)
    override var u: Double = update(0.0)

    override fun update(dt: Double): Double {
        val targetPos = bounds.apply(t)

        val diff = targetPos-x.estimatedPos
        val dx = diff.map { min(it.absoluteValue,dt*maxSpeed.value)*it.sign }

        x = ServoControlLoopState(x.estimatedPos+dx,0.rad/s)

        var theta: Expression = t
        if(theta.value.isNaN()) {
            println("WARNING: SERVO POSITION IS NaN")
            return Double.NaN
        }
        while (theta<bounds.min) theta += 1.revolutions
        while (theta>bounds.max) theta -= 1.revolutions

        //cannot reach, choose closest bound
        if(theta<bounds.min) {
            println("WARNING: SERVO TARGET IS OUTSIDE OF BOUNDS")
            val minDist = (t-bounds.min).normalizeRadian().map { it.absoluteValue }
            val maxDist = (t-bounds.max).normalizeRadian().map { it.absoluteValue }
            theta = if(minDist < maxDist) bounds.min else bounds.max
        }

        val servoPos =  (posMap)(theta.value).toDouble()

        return servoPos
    }

    // feedback
    override fun read() = x

    override fun reached(x: ServoControlLoopState, t: Radian)
            = (x.estimatedPos-bounds.apply(t)).map { it.absoluteValue } < tolerance

    override fun write(u: Double) {
        servo.updatePort(u)
        servo.node.tickControlNode(0.0)
    }
}
