package sigmacorns.common.io

import net.unnamedrobotics.lib.rerun.RerunConnection
import sigmacorns.common.RobotTickI
import sigmacorns.common.RobotTickO
import java.io.Closeable

interface SigmaIO: Closeable {
    val rerunConnection: RerunConnection
    fun update(o: RobotTickO): RobotTickI

    override fun close() {
        rerunConnection.close()
    }
}