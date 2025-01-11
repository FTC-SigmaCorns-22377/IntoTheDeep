package sigmacorns.common


import android.os.Environment
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.rerun.rerun
import org.junit.Test
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ArmPose
import sigmacorns.common.subsystems.path.ChoreoController
import sigmacorns.common.subsystems.path.choreoControllerLoop
import sigmacorns.common.subsystems.swerve.ModuleController
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import java.io.File

class ChoreoTest {

    @Test
    fun test() {
        Choreo::class.java.getDeclaredField("CHOREO_DIR").let {
            it.isAccessible = true
            it.set(null,File("C:\\Users\\chemi\\Documents\\choreo"))
        }

        val pos = Transform2D(0.m,0.m,0.rad)

        val io = SimIO(
            initialPos = pos,
            initialArmPose = ArmPose(pos.vector(),pos.angle.cast(rad),400.mm,0.rad,0.rad,0.rad)
        )

        val robot = Robot(io)

        val swerveController = SwerveController(ModuleController(
            PIDCoefficients(0.05,0.0,0.0)
        ),robot.drivebase)
        val choreoController = ChoreoController(10.0,1.0)

        val swerveLoop = swerveControlLoop(swerveController)
        val choreoLoop = choreoControllerLoop(choreoController,swerveLoop)
        io.addLoop(swerveLoop)
        io.addLoop(choreoLoop)

        val traj = (Choreo::loadTrajectory)("New Path").get() as Trajectory<SwerveSample>

        runBlocking { choreoLoop.target(traj) }

        runBlocking {
            robot.ioLoop { it > 10.s }
        }
//        robot.launchIOLoop { it > 10.s }

//        runBlocking { robot.ioLoop?.join() }
    }
}