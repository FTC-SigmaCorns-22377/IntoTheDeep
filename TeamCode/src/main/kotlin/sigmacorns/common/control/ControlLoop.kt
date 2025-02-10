package sigmacorns.common.control

import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.control.circuit.ControlNode
import net.unnamedrobotics.lib.control.circuit.PortIn
import net.unnamedrobotics.lib.control.circuit.PortOut
import net.unnamedrobotics.lib.control.circuit.PortTIn
import net.unnamedrobotics.lib.control.circuit.PortUOut
import net.unnamedrobotics.lib.control.circuit.PortXOut
import net.unnamedrobotics.lib.control.circuit.UPortKind
import net.unnamedrobotics.lib.control.circuit.XPortKind
import net.unnamedrobotics.lib.control.controller.Controller
import sigmacorns.common.io.SigmaIO

abstract class ControlLoop<X:Any, U: Any, T: Any>(
    name: String,
    val io: SigmaIO,
): PortXOut<X>, PortTIn<T>, PortUOut<U>, ControlNode {
    var disabled = false

    abstract fun update(deltaTime: Double): U

    abstract var x: X
    abstract var u: U
    abstract var t: T

    abstract fun read(): X
    abstract fun write(u: U)
    abstract fun reached(x: X, t: T): Boolean

    fun reached() = reached(x,t)

    override fun getXOut() = x
    override fun updateTIn(v: T) { t = v }
    override fun getUOut() = u

    override fun tickControlNode(dt: Double) {
        if(disabled) return
        x = read()
        write(update(dt))
    }

    fun follow(target: T) = follow { target }
    fun follow(target: () -> T) = cmd {
        var t: T? = null
        init { t = target(); this@ControlLoop.t = t!!; }
        finishWhen { reached(x,t!!) }
    }
}


fun <X: Any,U: Any,T: Any> Controller<X,U,T>.toControlLoop(
    name: String,
    io: SigmaIO,
    fRead: context(ControlLoop<X,U,T>) () -> X,
    fWrite: context(ControlLoop<X,U,T>) (U) -> Unit,
    fReached: context(ControlLoop<X,U,T>) (X,T) -> Boolean = { _,_ -> true }
) = object : ControlLoop<X,U,T>(name,io) {
    override fun update(deltaTime: Double): U {
        return updateStateless(deltaTime,x,t)
    }

    override var x: X
        get() = position
        set(value) { position = value }
    override var u: U
        get() = output
        set(value) { output = value }
    override var t: T
        get() = target
        set(value) { target = value }

    override fun read() = fRead(this)
    override fun write(u: U) = fWrite(this,u)

    override fun reached(x: X, t: T) = fReached(this,x,t)
}

context(SigmaIO)
class Sensor<T: Any>(
    private val f: context(SigmaIO) () -> T,
): PortOut<T, XPortKind>() {
    private lateinit var t: T

    override val node: ControlNode = object :ControlNode {
        override fun tickControlNode(dt: Double) {
            t = f(this@SigmaIO)
        }
    }

    override fun getPort() = t
}

context(SigmaIO)
class Actuator<T: Any>(
    private val f: context(SigmaIO) (T) -> Unit,
): PortIn<T, UPortKind>() {
    private var v: T? = null

    override val node = object : ControlNode {
        override fun tickControlNode(dt: Double): Unit {
            v?.let { f(this@SigmaIO,it) }
        }
    }

    override fun updatePort(v: T) { this.v = v }
}