package sigmacorns.common.io

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.rerun.RerunConnection
import java.io.Closeable

abstract class SigmaIO: Closeable {
    abstract val rerunConnection: RerunConnection

    override fun close() {
        rerunConnection.close()
    }

    abstract fun updateSensors()
    abstract fun resetSlideMotors(zero1: Tick, zero2: Tick)

    abstract fun setPinPos(p: Transform2D)
    abstract fun position(): Transform2D
    abstract fun velocity(): Twist2D

    abstract fun motor1Pos(): Tick
    abstract fun motor2Pos(): Tick

    abstract fun intakeLimitTriggered(): Boolean
    abstract fun liftLimitTriggered(): Boolean

    abstract fun intakeCurrent(): Expression

    abstract fun red(): Int
    abstract fun green(): Int
    abstract fun blue(): Int
    abstract fun alpha(): Int
    abstract fun distance(): Metre

    abstract fun voltage(): Volt
    abstract fun time(): Second

    abstract var driveFL: Double
    abstract var driveBL: Double
    abstract var driveBR: Double
    abstract var driveFR: Double

    abstract var motor1: Double
    abstract var motor2: Double
    abstract var motor3: Double

    abstract var intake: Double
    abstract var push: Double

    abstract var armL: Double
    abstract var armR: Double
    abstract var wrist: Double

    abstract var claw: Double

    abstract var flap: Double

    abstract var tilt1: Double
    abstract var tilt2: Double

    abstract var pto1: Double
    abstract var pto2: Double
}