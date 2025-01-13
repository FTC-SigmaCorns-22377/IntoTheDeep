package sigmacorns.common


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
import net.unnamedrobotics.lib.math2.vec2
import org.junit.Test
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ScoringPose
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

        val traj = (Choreo::loadTrajectory)("autoish").get() as Trajectory<SwerveSample>

        val pos = traj.initialSample.let { Transform2D(it.x.m,it.y.m,it.heading.rad) }

        val io = SimIO(
            initialPos = pos,
            initialScoringPose = ScoringPose(pos.vector(),pos.angle.cast(rad),400.mm,0.rad,0.rad,0.rad)
        )

        val robot = Robot(io)

        val swerveController = SwerveController(ModuleController(
            Tuning.SWERVE_MODULE_PID
        ),robot.drivebase)
        val choreoController = ChoreoController(
            PIDCoefficients(12.0,0.4,4.0),
            PIDCoefficients(12.0,0.3,2.0),
            vec2(robot.drivebase.width,robot.drivebase.length),3
        )

        val swerveLoop = swerveControlLoop(swerveController)
        val choreoLoop = choreoControllerLoop(choreoController,swerveLoop)
        io.addLoop(swerveLoop)
        io.addLoop(choreoLoop)

        runBlocking { choreoLoop.target(traj) }

        runBlocking {
            robot.ioLoop { it > 10.s }
        }
    }
}