package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.unnamedrobotics.lib.math2.Transform2D
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ScoringPose

var SIM = false

abstract class SimOrHardwareOpMode: LinearOpMode() {
    abstract fun runOpMode(io: SigmaIO)

    abstract val initialScoringPose: ScoringPose

    open val rerunName: String = "unnamed"

    var io: SigmaIO? = null

    override fun runOpMode() {
        io = if (SIM)
            SimIO(
                initialPos = initialScoringPose.let {
                    Transform2D(it.robotPos,it.theta)
                },
                initialScoringPose = initialScoringPose,
                rerunName = rerunName
            )
        else
            RobotIO(
                hardwareMap,
                rerunName = rerunName,
                initialScoringPose = initialScoringPose

            )

        runOpMode(io!!)
    }
}