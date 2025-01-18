package sigmacorns.common.cmd

import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.CommandBuilder
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.forever
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.instant
import net.unnamedrobotics.lib.command.wait
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import sigmacorns.common.Constants
import sigmacorns.common.Robot
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringTarget
import sigmacorns.common.subsystems.path.choreoControllerLoop
import sigmacorns.common.subsystems.swerve.SwerveHeadingController
import sigmacorns.common.subsystems.swerve.swerveHeadingControlLoop

fun Robot.hover(
    trajName: String,
    pos: Vector3,
    yaw: Radian,
    pitch: Radian = 179.degrees,
    posThreshold: Metre = 10.cm,
    velThreshold: Expression = 5.cm/s,
) = cmd {
    val traj = (Choreo::loadTrajectory)(trajName).get() as Trajectory<SwerveSample>
    val endPos = traj.finalPose.let { Transform2D(it.getX(DistanceUnit.METER).m,it.getY(DistanceUnit.METER).m,it.getHeading(AngleUnit.RADIANS).rad) }
    val targetState = ScoringKinematics.inverse(ScoringTarget(endPos.vector(),endPos.angle.cast(rad),pos,pitch,yaw))

    init { runBlocking {
        choreoControlLoop.enabled = true
        choreoControlLoop.target(traj)
        armControlLoop.target(targetState.armTarget(true))
    } }

    finishWhen {
        val curPos = swerveControlLoop.controller.logPosition
        (endPos - curPos).vector().magnitude() < posThreshold &&
                swerveControlLoop.controller.logVelocity.vector().magnitude() < velThreshold &&
                ScoringKinematics.inverse(ScoringTarget(curPos.vector(),curPos.angle.cast(rad),pos,pitch,yaw)).let {
                    Constants.ARM_PIVOT_BOUNDS.apply(it.pivot) == it.pivot &&
                            Constants.ARM_EXTENSION_BOUNDS.apply(it.extension) == it.extension &&
                            Constants.CLAW_ROLL_BOUNDS.apply(it.roll) == it.roll &&
                            Constants.CLAW_PITCH_BOUNDS.apply(it.pitch) == it.pitch
                }
    }

    onFinish {
        val curPos = swerveControlLoop.controller.logPosition
        val curScoringTarget = ScoringTarget(curPos.vector(),curPos.angle.cast(rad),pos,pitch,yaw)
        val curScoringPose = ScoringKinematics.inverse(curScoringTarget)

        choreoControlLoop.enabled = false
        swerveControlLoop.enabled = false
        swerveHeadingControlLoop.enabled = true
        swerveHeadingControlLoop.target(SwerveHeadingController.Target(vec2(0.m/s,0.m/s), curScoringPose.theta.cast(rad)))
        armControlLoop.target(curScoringPose.armTarget(true))
    }
}

infix fun Command.race(rhs: Command) = cmd {
    init {
        this@race.init()
        rhs.init()
    }

    run {
        this@race.run() || rhs.run()
    }
}