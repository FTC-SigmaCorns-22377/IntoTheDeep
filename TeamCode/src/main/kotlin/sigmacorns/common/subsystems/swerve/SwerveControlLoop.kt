package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.revolution
import sigmacorns.common.LOGGING
import sigmacorns.common.LoopTimes
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO

fun swerveControlLoop(controller: SwerveController) = ControlLoopContext(
    LoopTimes.SWERVE,
    controller,
    { io: SigmaIO ->
        SwerveController.State(io.turnVoltages().map { (it/(3.3.V)* revolution).cast(rad) })
    },
    { u: SwerveController.Input, io: SigmaIO ->
        io.drivePowers.zip(u.drivePowers).forEach { it.first.write(it.second) }
        io.turnPowers.zip(u.turnPowers).forEach { it.first.write(it.second) }
    },
    { if(LOGGING.RERUN_SWERVE) it.log("swerve") },
    name = "swerve"
)