package sigmacorns.common.io

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.driver.gobilda.GoBildaPose2D
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

    abstract fun position(): Transform2D
    abstract fun velocity(): Twist2D
    abstract fun voltage(): Volt
    abstract fun time(): Second
}