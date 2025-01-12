package sigmacorns.common.swerve

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.div
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
import sigmacorns.common.subsystems.swerve.SwerveAccelerationController
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveAccLogPosControlLoop
import sigmacorns.common.subsystems.swerve.swerveAccelerationControllerLoop
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import sigmacorns.common.subsystems.swerve.swerveLogPosControlLoop
import sigmacorns.common.subsystems.swerve.swerveLogVelControlLoop

class AccTest {
    @Test
    fun test() {
        val pos = Transform2D(0.m,0.m,0.rad)

        val io = SimIO(
            initialPos = pos,
            initialArmPose = ArmPose(pos.vector(),pos.angle.cast(rad),400.mm,0.rad,0.rad,0.rad)
        )

        val robot = Robot(io)

        val swerveController = SwerveAccelerationController(robot.drivebase)

        val swerveLoop = swerveAccelerationControllerLoop(swerveController)
        io.addLoop(swerveLoop)
        io.addLoop(swerveAccLogPosControlLoop(swerveLoop))

        runBlocking {
            swerveLoop.target(SwerveAccelerationController.Target(
                Transform2D((1.0).m/ s/s,0.m/ s/s,0.rad/ s/s)))
            robot.ioLoop { it > 10.s }
        }
    }
}