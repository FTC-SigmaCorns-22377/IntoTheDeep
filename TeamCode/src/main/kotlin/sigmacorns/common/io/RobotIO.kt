package sigmacorns.common.io

import com.qualcomm.robotcore.hardware.HardwareMap
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.rerun.RerunConnection

class RobotIO(
    val hardwareMap: HardwareMap,
    io: String = "192.168.43.122",
    rerunName: String = "unnamed",
): SigmaIO() {
    override val rerunConnection: RerunConnection
        get() = TODO("Not yet implemented")

    override fun position(): Transform2D {
        TODO("Not yet implemented")
    }

    override fun velocity(): Twist2D {
        TODO("Not yet implemented")
    }

    override fun voltage(): Volt {
        TODO("Not yet implemented")
    }

    override fun time(): Second {
        TODO("Not yet implemented")
    }
}