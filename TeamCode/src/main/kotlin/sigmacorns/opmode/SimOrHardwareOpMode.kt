package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.unnamedrobotics.lib.math2.Transform2D
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.io.SimIO

var SIM = false

abstract class SimOrHardwareOpMode: LinearOpMode() {
    abstract fun runOpMode(io: SigmaIO)

    open val rerunName: String = "unnamed"

    var io: SigmaIO? = null

    override fun runOpMode() {
        io = if (SIM)
            SimIO()
        else
            RobotIO(
                hardwareMap,
                rerunName = rerunName,
            )

        runOpMode(io!!)
    }
}