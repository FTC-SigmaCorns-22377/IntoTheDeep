package sigmacorns.common.swerve

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import org.junit.Test
import sigmacorns.common.Robot
import sigmacorns.common.Tuning
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ArmPose
import sigmacorns.common.subsystems.swerve.ModuleController
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import sigmacorns.common.subsystems.swerve.swerveLogPosControlLoop
import sigmacorns.common.subsystems.swerve.swerveLogVelControlLoop

class SwerbTest {
    @Test
    fun test() {
        val pos = Transform2D(0.m,0.m,0.rad)

        val io = SimIO(
            initialPos = pos,
            initialArmPose = ArmPose(pos.vector(),pos.angle.cast(rad),400.mm,0.rad,0.rad,0.rad)
        )

        val robot = Robot(io)

        val swerveController = SwerveController(
            ModuleController(
            Tuning.SWERVE_MODULE_PID
        ),robot.drivebase)

        val swerveLoop = swerveControlLoop(swerveController)
        io.addLoop(swerveLoop)
        io.addLoop(swerveLogPosControlLoop(swerveLoop))
        io.addLoop(swerveLogVelControlLoop(swerveLoop))

        runBlocking {
            swerveLoop.target(SwerveController.Target(Transform2D(0.m/s,0.m/s,1.rad/s),false))
            robot.ioLoop { it > 10.s }
        }
    }
}