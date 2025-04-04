package sigmacorns.common.control

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.rotate
import net.unnamedrobotics.lib.math2.vec2
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.common.Robot
import sigmacorns.constants.Tuning
import java.util.Optional
import kotlin.math.absoluteValue

data class RobotMovementState(val pos: Transform2D, val vel: Twist2D)

class ChoreoController(
    coeff: PIDCoefficients,
    angCoeff: PIDCoefficients,
    drivebaseSize: Vector2,
    val rerunDownscale: Int = 1
) : Controller<RobotMovementState, Transform2D, Optional<Trajectory<SwerveSample>>>() {
    override var output: Transform2D = Transform2D(0.m / s, 0.m / s, 0.rad / s)

    override var position: RobotMovementState = RobotMovementState(
        Transform2D(0.m, 0.m, 0.rad),
        Twist2D(0.m / s, 0.m / s, 0.rad / s)
    )

    override var target: Optional<Trajectory<SwerveSample>> = Optional.empty()
        set(value) {
            targetNew = true
            field = value
            tPreset = false
        }

    var targetNew = false
    var tPreset = false

    val trajectoryLogger = TrajectoryLogger(drivebaseSize.x.cast(m), drivebaseSize.y.cast(m))

    var lastTraj: Trajectory<SwerveSample>? = null

    var t = 0.0

    lateinit var sample: SwerveSample

    val xController = PIDController(coeff)
    val yController = PIDController(coeff)
    val angController = PIDController(angCoeff)

    override fun copy() = TODO()

    override fun update(deltaTime: Double): Transform2D {
        if (!target.isPresent) return Transform2D(0.m / s, 0.m / s, 0.rad / s)

        targetNew = false
        val target = target.get()
        if (lastTraj == target || tPreset) t += deltaTime else t = 0.0
        lastTraj = target

//        sample = target.getClosestSample(position.pos.toPose2d())
//        sample = target.sampleAt(sample.t + 1.ms.value)

        sample = target.sampleAt(t)
        val endSample = target.finalSample.pose

//        if((sample.pose.toTransform2d() - endSample.toTransform2d()).vector().magnitude() < 5.cm)
//            sample = target.finalSample

        val posErr = sample.pose.toTransform2d() - position.pos
        val robotRelPosErr = posErr.vector().rotate((-position.pos.angle).cast(rad)) / s
        val headingErr = posErr.angle / s

        val acc = Twist2D(
            vec2(sample.ax.m / s, sample.ay.m / s).rotate(
                (-position.pos.angle).cast(
                    rad
                )
            ), sample.alpha.rad / s
        )

        val velBase = Transform2D(
            vec2(sample.vx.m / s, sample.vy.m / s).rotate((-position.pos.angle).cast(rad)),
            sample.omega.rad / s
        )
//        val velErr = (velBase - position.vel.let { Twist2D(it.vector().rotate((-position.pos.angle).cast(rad)),it.dAngle) })

//        val res =
//            Transform2D(robotRelPosErr*coeff.p, headingErr*angCoeff.p) +
//            (velBase + Twist2D(velErr.vector()*coeff.d,velErr.dAngle*angCoeff.d)).exp() +
//            Twist2D(acc.vector()*coeff.i,acc.dAngle*angCoeff.i).exp()

        val res = Transform2D(
            vec2(
                xController.updateStateless(deltaTime, 0.0, posErr.x.value).m / s,
                yController.updateStateless(deltaTime, 0.0, posErr.y.value).m / s
            ).rotate((-position.pos.angle).cast(rad)),
            angController.updateStateless(deltaTime, 0.0, posErr.angle.value).rad / s
        ) + velBase

        return res
    }
}

fun choreoControllerLoop(
    choreoController: ChoreoController,
    robot: Robot
) = controlLoop(choreoController, "choreo", robot.io, {
    RobotMovementState(robot.io.position(), robot.io.velocity())
}, {
    robot.mecanum.t = it
}, { x: RobotMovementState, t ->
    if (!t.isPresent || choreoController.targetNew) return@controlLoop false

    val f = t.get().finalSample
    var log = "WAITING FOR: "
    val posReached = (x.pos - f.pose.toTransform2d()).let { d ->
        (d.vector().magnitude() < Tuning.choreoPosThresh)
            .also { if (!it) log += "POS(${d.vector().magnitude()}), " } &&
                (d.angle.normalizeRadian().map { it.absoluteValue } < Tuning.choreoAngThresh)
                    .also { if (!it) log += "ANG(${d.angle.normalizeRadian()}), " }
    }

    val velReached = ((x.vel.vector() - vec2(
        f.vx.m / s,
        f.vy.m / s
    )).magnitude() < Tuning.choreoVelThresh)
        .also { log += "VEL(${x.vel.vector().magnitude()}), " }
    val angVelReached =
        ((x.vel.dAngle - f.omega.rad / s).map { it.absoluteValue } < Tuning.choreoAngVelThresh)
            .also { log += "ANGVEL(${x.vel.dAngle}), " }

    println(log)
    posReached
})

fun Transform2D.toPose2d(): Pose2D =
    Pose2D(DistanceUnit.METER, x.value, y.value, AngleUnit.RADIANS, angle.value)

fun Pose2D.toTransform2d(): Transform2D = Transform2D(
    getX(DistanceUnit.METER).m,
    getY(DistanceUnit.METER).m,
    getHeading(AngleUnit.RADIANS).rad
)