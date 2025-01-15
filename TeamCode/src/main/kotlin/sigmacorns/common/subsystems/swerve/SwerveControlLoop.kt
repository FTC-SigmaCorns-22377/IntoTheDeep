package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.sync.withLock
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.revolution
import sigmacorns.common.Constants.MODULE_OFFSET
import sigmacorns.common.LOGGING
import sigmacorns.common.LoopTimes
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO

fun swerveControlLoop(controller: SwerveController) = ControlLoopContext(
    LoopTimes.SWERVE,
    controller,
    { io: SigmaIO ->
        SwerveController.State(
            io.turnVoltages()
                .map { (it/(3.3.V)* revolution) }
                .zip(MODULE_OFFSET)
                .map { (it.first - it.second).cast(rad) }
        )
    },
    { u: SwerveController.Input, io: SigmaIO ->
        io.drivePowers.zip(u.drivePowers).forEach { it.first.write(it.second) }
        io.turnPowers.zip(u.turnPowers).forEach { it.first.write(it.second) }
    },
    { if(LOGGING.RERUN_SWERVE) it.log("swerve") },
    name = "swerve"
)

fun <T: Any> idController() = object: Controller<T, T, Int>() {
    override lateinit var output: T
    override lateinit var position: T
    override var target: Int = 0

    override fun copy() = TODO()

    override fun update(deltaTime: Double) = position

}

fun swerveLogPosControlLoop(loop: ControlLoopContext<*,*,*,*,SwerveController>) = ControlLoopContext(
    LoopTimes.SWERVE_POS_UPDATE,
    idController(),
    { io: SigmaIO -> io.position() },
    { u: Transform2D, io -> loop.withLock {
        loop.controller.logPosition = u
    }},
    name = "swerveLogPos"
)
fun swerveLogVelControlLoop(loop: ControlLoopContext<*,*,*,*,SwerveController>) = ControlLoopContext(
    LoopTimes.SWERVE_POS_UPDATE,
    idController(),
    { io: SigmaIO -> io.velocity() },
    { u: Twist2D, io -> loop.withLock {
        loop.controller.logVelocity = u
    } },
)

fun swerveAccelerationControllerLoop(swerveAccelerationController: SwerveAccelerationController) = ControlLoopContext(
    LoopTimes.SWERVE,
    swerveAccelerationController,
    { io: SigmaIO ->
        SwerveAccelerationController.State(io.turnVoltages().map { (it/(3.3.V)* revolution).cast(rad) },io.velocity())
    },
    { u: SwerveAccelerationController.Input, io: SigmaIO ->
        io.drivePowers.zip(u.drivePowers).forEach { it.first.write(it.second) }
        io.turnPowers.zip(u.turnPowers).forEach { it.first.write(it.second) }
    },
    { it.log("swerve") },
    name = "swerveAcc"
)
fun swerveAccLogPosControlLoop(loop: ControlLoopContext<*,*,*,*,SwerveAccelerationController>) = ControlLoopContext(
    LoopTimes.SWERVE_POS_UPDATE,
    idController(),
    { io: SigmaIO -> io.position() },
    { u: Transform2D, io -> loop.withLock {
        loop.controller.logPosition = u
    }},
)
