package sigmacorns.common.io

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunSources
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.sim.SimInput
import sigmacorns.common.sim.SimNative
import sigmacorns.constants.SimLoopTimes
import kotlin.math.floor
import kotlin.math.max
import kotlin.random.Random

class SimIO(
    private val sim: SimNative,
    rerunName: String,
    val simV: Volt = 12.V,
    val realtime: Boolean = false
): SigmaIO() {
    override val rerunConnection = RerunConnection(rerunName,RerunSources.TCP(rerunName))

    override fun updateSensors() {}
    override fun resetSlideMotors(zero1: Tick, zero2: Tick) {
        TODO("Not yet implemented")
    }

    override fun setPinPos(p: Transform2D) {
    }

    val simStep: Second = 2.ms
    private var realStartTime: Expression = Clock.seconds.s
    private var time: Expression = 0.s

    @Suppress("NAME_SHADOWING")
    private fun step(dt: Expression) {
        val estimatedDt =
            (dt + dt*Random.nextDouble(-SimLoopTimes.uncertainty,SimLoopTimes.uncertainty)).map {
                max(it,SimLoopTimes.base.value)
            }
        val dt = if(realtime)
                (Clock.seconds.s-realStartTime)-time
            else estimatedDt

        time += dt

        val numSteps = floor((dt/simStep).value).toInt()
        val input = SimInput(
            driveFL,driveBL,driveBR,driveFR,motor1,motor2
        )

        repeat(numSteps) {
            sim.step(simStep.value, input)
        }

        val rem = dt-numSteps*simStep
        sim.step(rem.value,input)

    }

    override fun position() = sim.output.pos.also { step(SimLoopTimes.pinpointRead) }
    override fun velocity() = sim.output.vel
    override fun motor1Pos(): Tick = sim.output.motor1.tick

    override fun motor2Pos(): Tick = sim.output.motor2.tick
    override fun intakeLimitTriggered(): Boolean {
        TODO("Not yet implemented")
    }

    override fun liftLimitTriggered(): Boolean {
        TODO("Not yet implemented")
    }

    override fun intakeCurrent(): Expression {
        return 0.A
    }

    override fun red(): Int {
        return 0
    }

    override fun green(): Int {
        return 0
    }

    override fun blue(): Int {
        return 0
    }

    override fun alpha(): Int {
        return 0
    }

    var triggerDistance = false

    override fun distance(): Metre
        = (if(time>5.s) 20.cm else 20.cm).also { step(SimLoopTimes.motorWrite) }

    override fun voltage() = simV

    override fun time() = time.cast(s).also { step(SimLoopTimes.base) }

    override var driveFL: Double = 0.0
        set(value) {
            field = value
            step(SimLoopTimes.motorWrite)
        }
    override var driveBL: Double = 0.0
        set(value) {
            field = value
            step(SimLoopTimes.motorWrite)
        }
    override var driveBR: Double = 0.0
        set(value) {
            field = value
            step(SimLoopTimes.motorWrite)
        }
    override var driveFR: Double = 0.0
        set(value) {
            field = value
            step(SimLoopTimes.motorWrite)
        }
    override var motor1: Double = 0.0
        set(value) {
            field = value
            step(SimLoopTimes.motorWrite)
        }
    override var motor2: Double = 0.0
        set(value) {
            field = value
            step(SimLoopTimes.motorWrite)
        }
    override var motor3: Double
        get() = TODO("Not yet implemented")
        set(value) {}
    override var intake: Double = 0.0
        set(value) {
            println("INTAKE=$value")
            field = value
            step(SimLoopTimes.motorWrite)
        }


    override var armL: Double = 0.0
    override var armR: Double = 0.0
    override var wrist: Double
        get() = TODO("Not yet implemented")
        set(value) {}
    override var claw: Double = 0.0
    override var flap: Double = 0.0
    override var push: Double = 0.0
    override var tilt1: Double = 0.0
    override var tilt2: Double = 0.0
    override var pto1: Double
        get() = TODO("Not yet implemented")
        set(value) {}
    override var pto2: Double
        get() = TODO("Not yet implemented")
        set(value) {}
}