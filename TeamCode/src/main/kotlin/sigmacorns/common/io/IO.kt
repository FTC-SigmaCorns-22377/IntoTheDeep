package sigmacorns.common.io

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Hertz
import net.unnamedrobotics.lib.control.controller.Controller
import kotlinx.coroutines.*
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import sigmacorns.common.LOGGING
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.absoluteValue

/**
 * @param fWrite Will be run in the same thread as BaseIO.
 * @param fRead Will be run in the same thread as BaseIO.
 */
data class ControlLoopContext<X,U,T: Any,IO: BaseIO<IO>, C: Controller<X,U,T>>(
    var targetUpdate: Second,
    internal val controller: C,
    private val fRead: suspend context(ControlLoopContext<X,U,T, IO,C>) (IO) -> X,
    private val fWrite: suspend context(ControlLoopContext<X,U,T, IO,C>) (U,IO) -> Unit,
    private val fLog: suspend context(RerunPrefix, RerunConnection) (C) -> Unit = {},
    var enabled: Boolean = true,
    var write: AtomicBoolean = AtomicBoolean(true),
    val name: String = "unnamed",
    val async: Boolean = false
) {
    private val lock = Mutex()

    constructor(
        hz: Hertz,
        controller: C,
        fRead: suspend context(ControlLoopContext<X, U, T, IO,C>) (IO) -> X,
        fWrite: suspend context(ControlLoopContext<X, U, T, IO,C>) (U, IO) -> Unit,
        fLog: context(RerunPrefix, RerunConnection) (C) -> Unit = {},
        enabled: Boolean = true,
        write: AtomicBoolean = AtomicBoolean(true),
        name: String = "unnamed",
        async: Boolean = false
    ): this(hz.pow(-1).cast(s), controller, fRead, fWrite, fLog,enabled,write,name,async)

    @Volatile
    var status: Status = Status.INIT

    internal var neededForUpdate: MutableSet<Sensor<*>> = mutableSetOf()
    internal var lastUpdate: Second = Double.MIN_VALUE.s

    enum class Status { INIT, IDLE, COMPUTING, DONE, LOGGING }

    suspend fun <T> withLock(f: suspend () -> T): T = if(async) lock.withLock { f() } else f()

    internal suspend fun read(io: IO): X = withLock {
        if(LOGGING.LOG_IO) println("IO read for $name control loop.")
        val x = fRead(this,io)
        if(LOGGING.LOG_IO) println("Loop $name needs ${neededForUpdate.map { it.name }} to update")
        x
    }

    internal suspend fun write(io: IO) = withLock {
        if(LOGGING.LOG_IO) println("IO written for $name control loop.")
        if(write.get()) fWrite(this,controller.output,io)
    }

    internal suspend fun log(connection: RerunConnection) = withLock {
        if(LOGGING.LOG_IO) println("Logged $name control loop.")
            fLog(object: RerunPrefix {
                override val path: String = "controlLoops"
                context(RerunPrefix, RerunConnection) override fun log(name: String) {}
            }, connection, controller)
    }

    suspend fun target(t: T) = withLock {
        if(LOGGING.LOG_IO) println("Target set for $name control loop.")
        controller.target = t
    }

    fun mapTarget(f: (C) -> T) = runBlocking {
        withLock { controller.target = f(controller) }
    }
}

abstract class BaseIO<Self: BaseIO<Self>>(
    private val loops: MutableList<ControlLoopContext<*,*,*,Self,*>> = mutableListOf()
) {

    private val IOLock = Mutex()

    abstract val rerunConnection: RerunConnection
    abstract fun clearBulkCache()

    fun addLoop(it: ControlLoopContext<*,*,*,Self,*>) {
        loops += it
    }

    suspend fun run(t: Second) = coroutineScope {
        loops
            .filter { it.status == ControlLoopContext.Status.DONE && it.enabled }
            .forEach {
                it.write(this@BaseIO as Self)
                it.status = ControlLoopContext.Status.LOGGING
                it.log(rerunConnection)
                it.status = ControlLoopContext.Status.IDLE
            }

        var sensorToUpdate = null as Sensor<*>?
        for(loop in loops.filter { it.enabled }.sortedBy { -(t-it.lastUpdate)/it.targetUpdate }) {
            sensorToUpdate = loop.neededForUpdate.find { sensorShouldUpdate(t,it,loop) }
            if (sensorToUpdate != null) {
                break
            }
        }

        sensorToUpdate?.let { sensor ->
            sensor.update(t)
            if(sensor.bulkReadable) {
                clearBulkCache()
                loops
                    .filter { it.enabled }
                    .flatMap { it.neededForUpdate.filter { it.bulkReadable } }
                    .forEach { if(it!=sensor) it.update(t) }
            }
        }

        loops.filter { loopShouldUpdate(it) && it.enabled }
            .forEach {
                it.status = ControlLoopContext.Status.COMPUTING
                val dt = (t-it.lastUpdate)
                val x = it.read(this@BaseIO as Self)
                it.lastUpdate = t

                val f = suspend {
                    @Suppress("UNCHECKED_CAST")
                    it as ControlLoopContext<Any?,Any?,Any, Self,Controller<Any?,Any?,Any>>

                    it.withLock {
                        if(LOGGING.LOG_IO) println("Running ${it.name} control loop")
                        it.controller.output = it.controller.updateStateless(dt.value, x, it.controller.target)
                    }

                    // TODO: make rerun log here in the thread instead. need to make RerunConnection thread safe.

                    it.status = ControlLoopContext.Status.DONE
                }

                if(it.async) launch(Dispatchers.Default) { f() } else f()
            }

//        if(sensorToUpdate == null) yield()
    }

    private fun sensorShouldUpdate(t: Second, sensor: Sensor<*>, loop: ControlLoopContext<*,*,*, Self,*>): Boolean {
        val sensorStale = sensor.lastUpdate < t
        val sensorFreshForLoop = loop.lastUpdate >= sensor.lastUpdate
        val res = sensorStale && sensorFreshForLoop

        return res
    }

    private fun loopShouldUpdate(loop: ControlLoopContext<*, *, *, Self,*>)
        = loop.neededForUpdate.all { it.lastUpdate > loop.lastUpdate && it.status == Sensor.Status.IDLE }
}

fun <T: Any> sensor(bulkReadable: Boolean = false, async: Boolean = false, name: String = "unnamed", f: () -> T) = object: Sensor<T>() {
    override val bulkReadable: Boolean = bulkReadable
    override val async: Boolean = async
    override val name: String = name

    override suspend fun poll(): T {
        if(LOGGING.LOG_IO) println("Polling sensor $name")
        return f()
    }
}

abstract class Sensor<T: Any> {
    enum class Status { IDLE, READING }

    @Volatile
    private var x: T? = null

    var lastUpdate: Second = 0.s
    val status
        get()
        = if(job?.isCompleted != false)
            Status.IDLE
        else
            Status.READING

    private var job: Job? = null

    abstract val bulkReadable: Boolean
    abstract val async: Boolean
    abstract val name: String

    abstract suspend fun poll(): T

    internal suspend fun update(t: Second) = coroutineScope {
        lastUpdate = t

        if(async) job = launch(Dispatchers.IO) {
            x = poll()
        } else {
            x = poll()
        }
    }

    context(ControlLoopContext<*,*,*,*,*>)
    fun get(): T {
        if(x==null) x = runBlocking { poll() }
        neededForUpdate.add(this)
        return x!!
    }
}

fun interface Actuator<T: Any> {
    fun write(u: T)
}

fun cachedActuator(updateThreshold: Double, f: (Double) -> Unit) = object: CachedActuator() {
    override fun cachedWrite(u: Double) = f(u)
    override val updateThreshold = updateThreshold
}

abstract class CachedActuator: Actuator<Double> {
    abstract fun cachedWrite(u: Double)
    abstract val updateThreshold: Double
    private var last: Double? = null

    override fun write(u: Double) {
        cachedWrite(u)
//        if(last?.let {
//            (it-u).absoluteValue > updateThreshold
//        } != false) {
//            cachedWrite(u)
//            last = u
//        } else if (u.absoluteValue < updateThreshold*2 && last?.let { it != 0.0 } != false) {
//            cachedWrite(0.0)
//            last = 0.0
//        }
    }
}