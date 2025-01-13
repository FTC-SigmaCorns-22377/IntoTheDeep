package sigmacorns.common.io

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.driver.gobilda.GoBildaPose2D
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.rerun.RerunConnection
import java.io.Closeable

abstract class SigmaIO: Closeable, BaseIO<SigmaIO>() {
    override fun close() {
        rerunConnection.close()
    }

    context(ControlLoopContext<*,*,*,SigmaIO,*>)
    abstract fun armPositions(): List<Tick>

    context(ControlLoopContext<*,*,*,SigmaIO,*>)
    abstract fun turnVoltages(): List<Volt>

    context(ControlLoopContext<*,*,*,SigmaIO,*>)
    abstract fun position(): Transform2D

    context(ControlLoopContext<*,*,*,SigmaIO,*>)
    abstract fun velocity(): Twist2D

    abstract val armMotorPowers: List<Actuator<Double>>
    abstract val drivePowers: List<Actuator<Double>>
    abstract val turnPowers: List<Actuator<Double>>
    abstract val clawPos: Actuator<Double>
    abstract val diffyPos: List<Actuator<Double>>

    abstract fun voltage(): Volt
    abstract fun time(): Second
}