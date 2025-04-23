package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.rerun.RerunSources
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.io.SimIO
import sigmacorns.common.sim.SimNative
import java.time.Instant
import java.time.format.DateTimeFormatter
import java.util.Date

var SIM = false
var REALTIME = false

abstract class SimOrHardwareOpMode: LinearOpMode() {
    abstract fun runOpMode(io: SigmaIO)

    open val rerunName: String = "unnamed"
    open val rerunSrc: RerunSources
        get() = RerunSources.Disk("/sdcard/FIRST/rerun/$rerunName-${System.currentTimeMillis()/1000}.rrd")

    var io: SigmaIO? = null

    override fun runOpMode() {
        io = if (SIM)
            SimIO(SimNative(0.0,0.0, Transform2D(0.m,0.m,0.rad)), "SIM", realtime = REALTIME)
        else
            RobotIO(
                hardwareMap,
                rerunSrc = RerunSources.Disk("/sdcard/FIRST/rerun/$rerunName-${System.currentTimeMillis()/1000}.rrd"),
                rerunName = rerunName,
            )

        runOpMode(io!!)
    }
}