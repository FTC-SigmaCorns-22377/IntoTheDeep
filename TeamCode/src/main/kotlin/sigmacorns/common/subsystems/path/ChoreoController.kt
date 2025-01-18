package sigmacorns.common.subsystems.path

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.sync.withLock
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.rotate
import net.unnamedrobotics.lib.math2.vec2
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.common.LOGGING
import sigmacorns.common.LoopTimes
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.swerve.SwerveController
import kotlin.math.absoluteValue

data class RobotMovementState(val pos: Transform2D, val vel: Twist2D)

class ChoreoController(
    val coeff: PIDCoefficients,
    val angCoeff: PIDCoefficients,
    val drivebaseSize: Vector2,
    val rerunDownscale: Int = 1
): Controller<RobotMovementState, SwerveController.Target, Trajectory<SwerveSample>>() {
    override var output: SwerveController.Target = SwerveController.Target(Transform2D(0.m/s,0.m/s,0.rad/s),false)
    override lateinit var position: RobotMovementState
    override lateinit var target: Trajectory<SwerveSample>

    val trajectoryLogger = TrajectoryLogger(drivebaseSize.x.cast(m),drivebaseSize.y.cast(m))

    private var lastTraj: Trajectory<SwerveSample>? = null

    var t = 0.0

    lateinit var sample: SwerveSample

    override fun copy() = TODO()

    override fun update(deltaTime: Double): SwerveController.Target {
        if(lastTraj == target) t += deltaTime else t = 0.0
        lastTraj = target
//        sample = target.getClosestSample(position.pos.toPose2d())
//        sample = target.sampleAt(sample.t + 1.ms.value)
        sample = target.sampleAt(t)
        val endSample = target.finalSample.pose

        if((sample.pose.toTransform2d() - endSample.toTransform2d()).vector().magnitude() < 5.cm)
            sample = target.finalSample

        val posErr = sample.pose.toTransform2d()-position.pos
        val robotRelPosErr = posErr.vector().rotate((-position.pos.angle).cast(rad))/s
        val headingErr = posErr.angle/ s

        val acc = Twist2D(vec2(sample.ax.m/s,sample.ay.m/s).rotate((-position.pos.angle).cast(
            rad)),sample.alpha.rad/s)

        val velBase = Twist2D(
            vec2(sample.vx.m/s,sample.vy.m/s).rotate((-position.pos.angle).cast(rad)),
            sample.omega.rad/s
        )
        val velErr = (velBase - position.vel.let { Twist2D(it.vector().rotate((-position.pos.angle).cast(rad)),it.dAngle) })

        val res =
            Transform2D(robotRelPosErr*coeff.p, headingErr*angCoeff.p) +
            (velBase + Twist2D(velErr.vector()*coeff.d,velErr.dAngle*angCoeff.d)).exp() +
            Twist2D(acc.vector()*coeff.i,acc.dAngle*angCoeff.i).exp()

        return SwerveController.Target(res,false)

//        val acc = Transform2D(vec2(sample.ax.m/s,sample.ay.m/s).rotate((-position.pos.angle).cast(
//            rad)),sample.alpha.rad/s)*0.25
//
//        val posErr = sample.pose.toTransform2d()-position.pos
//        val robotRelPosErr = posErr.vector().rotate((-position.pos.angle).cast(rad))
//        val headingErr = posErr.angle
//
//        var d = Transform2D(
//            vec2(sample.vx.m/s,sample.vy.m/s)
//                .rotate((-position.pos.angle).cast(rad)),
//            sample.omega.rad/s*10
//        )*1.0
//
////        d *= (if(robotRelPosErr.magnitude()>getToPathRange || headingErr.value.absoluteValue > getToPathAngle.value) 0.0 else 1.0)
//
//        val vel = acc + d + Transform2D(robotRelPosErr*p/s,headingErr*pAng/s)
//
////        println("vel = (${vel.x.value},${vel.y.value})")
//
//        vel.x.cast(m/s)
//        vel.y.cast(m/s)
//        vel.angle.cast(rad/s)
//
//        return SwerveController.Target(vel,false)
    }
}

fun choreoControllerLoop(
    choreoController: ChoreoController, swerveLoop: ControlLoopContext<*,*,SwerveController.Target,*,SwerveController>
) = ControlLoopContext(
    LoopTimes.CHOREO,
    choreoController,
    { io: SigmaIO ->
        RobotMovementState(io.position(), io.velocity())
    },
    { u: SwerveController.Target, io: SigmaIO ->

//        println("U = (${u.vel.x.value}, ${u.vel.y.value})")
        swerveLoop.withLock {
            swerveLoop.controller.target = u
            swerveLoop.controller.logPosition = choreoController.position.pos
            swerveLoop.controller.logVelocity = choreoController.position.vel
        }
    },
    {
        if(LOGGING.RERUN_CHOREO) {
            it.trajectoryLogger.logTraj(it.target,TrajectoryLogger.Mode.VEL_ARROW)
            it.trajectoryLogger.highlightSample("closestSample",it.sample)
            it.trajectoryLogger.highlightSample("timeSample",it.target.sampleAt(it.t))
        }
    },
    name = "choreo"
)

private fun Transform2D.toPose2d(): Pose2D = Pose2D(DistanceUnit.METER, x.value, y.value, AngleUnit.RADIANS, angle.value)
private fun Pose2D.toTransform2d(): Transform2D = Transform2D(
    getX(DistanceUnit.METER).m,
    getY(DistanceUnit.METER).m,
    getHeading(AngleUnit.RADIANS).rad
)