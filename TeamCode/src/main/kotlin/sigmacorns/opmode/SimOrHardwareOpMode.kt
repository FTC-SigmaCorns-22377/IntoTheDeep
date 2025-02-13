package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.io.SimIO
import sigmacorns.common.sim.SimNative

var SIM = false
var REALTIME = false

abstract class SimOrHardwareOpMode: LinearOpMode() {
    abstract fun runOpMode(io: SigmaIO)

    open val rerunName: String = "unnamed"

    var io: SigmaIO? = null

    override fun runOpMode() {
        io = if (SIM)
            SimIO(SimNative(0.0,0.0, Transform2D(0.m,0.m,0.rad)), "SIM", realtime = REALTIME)
        else
            RobotIO(
                hardwareMap,
                rerunName = rerunName,
            )

        runOpMode(io!!)
    }
}