package sigmacorns.common.subsystems.arm

import net.unnamedrobotics.lib.math2.checkedUnitless
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.LOGGING.RERUN_ARM
import sigmacorns.common.LoopTimes
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO

fun armControlLoop() = ControlLoopContext(
    LoopTimes.ARM,
    ArmController(),
    { io: SigmaIO -> ArmState(io.armPositions().let { DiffyInputPose(it[0],it[1]) }) },
    { u: ArmInput, io: SigmaIO ->
        io.armMotorPowers.zip(u.motors).forEach { it.first.write((it.second.map { it.clampMagnitude(6) }/io.voltage()).checkedUnitless()) }
        io.diffyPos.zip(u.servoTarget).forEach { it.first.write(it.second.toDouble()) }
        io.clawPos.write(u.clawTarget);
    },
    { if(RERUN_ARM) it.log("robot/arm") },
    async = true,
)