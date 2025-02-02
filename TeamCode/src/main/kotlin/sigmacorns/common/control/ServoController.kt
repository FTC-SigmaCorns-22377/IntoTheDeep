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


data class ServoControlLoopState(val estimatedPos: Expression, val estimatedVel: Expression)

// TODO: change to estimator.
class ServoController(
    val servo: Actuator<Double>,
    val bounds: Bounds<Radian>,
    val maxAcc: Expression, // rad/s/s
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

        // distance to target
        val dx0 = (targetPos-x.estimatedPos)
        val dir = dx0.value.sign

        val v = x.estimatedVel
        val maxV = maxSpeed*dir
        val a = maxAcc*dir

        // distance it takes to accelerate/decelerate to/from v to maxV.
        // vf^2 = vi^2 + 2*a*d
        // d = (vf^2-vi^2)/2/a
        val dx1 = (maxV*maxV - v*v)/2.0/a

        // distance it takes to decelerate from v to 0.
        // vf^2 = vi^2 + 2a*d
        // 0 = maxV^2 + 2*(-a)*d
        // d = maxV^2/2/a
        val dx2 = v*v/a/2.0

        // distance before switching from acceleration to deceleration assuming no max speed.
        // explanation: it takes the same distance to go to/from estimated vel to maxV
        // then, theres just the distance it takes to decelerate from v to 0 (dx2).
        val dx3 = (dx0-dx2)/2.0

        val accDist = dx1.map { min(it,dx3.value) } // TODO
        val peakVel = (v*v + accDist*a*2).map { sqrt(it.absoluteValue)*dir }*s
        val decDist = accDist + dx2
        val steadyDist = dx0 - accDist - decDist

        // dx = dv/2.0*dt
        // dt = dx/dv*2.0
        val endAccT = (accDist/(peakVel-v)*2.0).takeUnless { it.value.isNaN() } ?: 0.s
        val startDecT = endAccT + steadyDist/peakVel
        val tf = startDecT + decDist/peakVel*2.0

        var newX = x.estimatedPos
        var newV = x.estimatedVel

        // acceleration
        min(dt,endAccT.value).s.let {
            // it = time accelerating
            newX += v*it + 1.0/2.0*a*it*it
            newV += it*a
        }

        // constant speed
        clamp(0.0, (dt.s-endAccT).value, startDecT.value).s.let {
            // it = time at a constant speed
            newX += it * peakVel
        }

        // deceleration
        clamp(0.0, (dt.s-startDecT).value, tf.value).s.let {
             // it = time decelerating
             newX += peakVel*it - 1.0/2.0*a*it*it
             newV -= it*a
        }

        x = ServoControlLoopState(newX,newV)

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
        = (x.estimatedPos-t).map { it.absoluteValue } < tolerance

    override fun write(u: Double) {
        servo.updatePort(u)
        servo.node.tickControlNode(0.0)
    }
}