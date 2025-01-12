package sigmacorns.common

import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import org.junit.Test
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ArmPose
import sigmacorns.common.subsystems.path.ChoreoAccController
import sigmacorns.common.subsystems.path.ChoreoController
import sigmacorns.common.subsystems.path.choreoAccControllerLoop
import sigmacorns.common.subsystems.path.choreoControllerLoop
import sigmacorns.common.subsystems.swerve.ModuleController
import sigmacorns.common.subsystems.swerve.SwerveAccelerationController
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveAccelerationControllerLoop
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import java.io.File

class ChoreoAccTest {
    @Test
    fun test() {
        Choreo::class.java.getDeclaredField("CHOREO_DIR").let {
            it.isAccessible = true
            it.set(null, File("C:\\Users\\chemi\\Documents\\choreo"))
        }

        val traj = (Choreo::loadTrajectory)("New Path").get() as Trajectory<SwerveSample>

        val pos = traj.initialSample.let { Transform2D(it.x.m,it.y.m,it.heading.rad) }

        val io = SimIO(
            initialPos = pos,
            initialArmPose = ArmPose(pos.vector(),pos.angle.cast(rad),400.mm,0.rad,0.rad,0.rad)
        )

        val robot = Robot(io)

        val swerveController = SwerveAccelerationController(robot.drivebase)
        val choreoController = ChoreoAccController(
            0.4,
            PIDCoefficients(2.0,0.0,20.0),
            PIDCoefficients(0.1,0.0,0.05),
            20.cm, 10.degrees, vec2(robot.drivebase.width,robot.drivebase.length),3
        )

        val swerveLoop = swerveAccelerationControllerLoop(swerveController)
        val choreoLoop = choreoAccControllerLoop(choreoController,swerveLoop)
        io.addLoop(swerveLoop)
        io.addLoop(choreoLoop)


        runBlocking { choreoLoop.target(traj) }

        runBlocking {
            robot.ioLoop { it > 10.s }
        }

//        runBlocking { robot.ioLoop?.join() }
    }
}