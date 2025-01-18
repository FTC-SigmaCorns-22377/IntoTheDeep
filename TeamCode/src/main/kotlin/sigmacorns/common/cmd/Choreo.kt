package sigmacorns.common.cmd

import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import sigmacorns.common.Robot
import kotlin.math.absoluteValue

fun Robot.follow(
    trajName: String,
    posThreshold: Metre = 8.cm,
    angleThreshold: Radian = 5.degrees,
    velThreshold: Expression = 5.cm/s,
    angVelThreshold: Expression = 10.degrees/s
) = cmd {
    val traj = (Choreo::loadTrajectory)(trajName).get() as Trajectory<SwerveSample>
    val endPos = traj.finalPose.let { Transform2D(it.getX(DistanceUnit.METER).m,it.getY(DistanceUnit.METER).m,it.getHeading(AngleUnit.RADIANS).rad) }

    init { runBlocking {
        choreoControlLoop.enabled = true
        choreoControlLoop.target(traj)
    } }

    finishWhen {
        val curPos = swerveControlLoop.controller.logPosition
        val curVel = swerveControlLoop.controller.logVelocity
        ((endPos - curPos).vector().magnitude() < posThreshold).also { if(!it) println("POS is not getting there. ${(endPos-curPos).vector().magnitude().value}.")}
                && (curVel.vector().magnitude() < velThreshold).also { if(!it) println("VEL is notg getting there")}
                && ((endPos - curPos).angle.normalizeRadian().map { it.absoluteValue } < angleThreshold).also { if(!it) println("Angle is not getting there")}
                && (curVel.dAngle.map { it.absoluteValue } < angVelThreshold).also { if(!it) println("ang velocity is having issues ")}
    }
}