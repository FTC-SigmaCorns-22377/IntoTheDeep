package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.path.ChoreoController
import sigmacorns.common.subsystems.path.choreoControllerLoop
import sigmacorns.common.subsystems.swerve.swerveLogPosControlLoop
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ChoreoTest: SimOrHardwareOpMode() {
    val traj = (Choreo::loadTrajectory)("Pickup 3").get() as Trajectory<SwerveSample>
    val pos = traj.initialSample.let { Transform2D(it.x.m,it.y.m,it.heading.rad) }
    override val initialScoringPose = ScoringPose(
        pos.vector(),pos.angle.cast(rad),
        406.4.mm,
        90.degrees,
        0.rad,0.rad
    )

    override fun runOpMode(io: SigmaIO) {
//        io.rerunConnection.disabled = true
        val robot = Robot(io)

        val choreoController = ChoreoController(
            PIDCoefficients(1.5,0.00,0.0),
            PIDCoefficients(2.0,0.0,0.0),
            vec2(robot.drivebase.width,robot.drivebase.length),3
        )

        val choreoLoop = choreoControllerLoop(choreoController,robot.swerveControlLoop)

        io.addLoop(robot.swerveControlLoop)
        io.addLoop(choreoLoop)
        io.addLoop(swerveLogPosControlLoop(robot.swerveControlLoop))

        waitForStart()

        robot.launchIOLoop()

        runBlocking { choreoLoop.target(traj) }

        while (opModeIsActive()) {
            idle()
        }
    }
}