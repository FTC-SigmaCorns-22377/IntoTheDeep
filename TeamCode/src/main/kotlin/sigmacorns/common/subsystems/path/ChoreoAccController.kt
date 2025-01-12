package sigmacorns.common.subsystems.path

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.sync.withLock
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Transform2D
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
import sigmacorns.common.subsystems.swerve.SwerveAccelerationController
import kotlin.math.absoluteValue

class ChoreoAccController(
    val a: Double,
    val coeff: PIDCoefficients,
    val angCoeff: PIDCoefficients,
    val getToPathRange: Metre,
    val getToPathAngle: Radian,
    val drivebaseSize: Vector2,
    val rerunDownscale: Int = 1
): Controller<RobotMovementState, SwerveAccelerationController.Target, Trajectory<SwerveSample>>() {
    override var output: SwerveAccelerationController.Target = SwerveAccelerationController.Target(
        Transform2D(0.m/s,0.m/s,0.rad/s))
    override lateinit var position: RobotMovementState
    override lateinit var target: Trajectory<SwerveSample>

    val trajectoryLogger = TrajectoryLogger(drivebaseSize.x.cast(m),drivebaseSize.y.cast(m))

    lateinit var sample: SwerveSample

    override fun copy() = ChoreoAccController(a,coeff, angCoeff, getToPathRange, getToPathAngle, drivebaseSize, rerunDownscale)

    override fun update(deltaTime: Double): SwerveAccelerationController.Target {
        sample = target.getClosestSample(position.pos.toPose2d())
        sample = target.sampleAt(sample.t + 3.ms.value)
        val endSample = target.finalSample.pose

        if((sample.pose.toTransform2d() - endSample.toTransform2d()).vector().magnitude() < 10.cm)
            sample = target.finalSample

        val posErr = sample.pose.toTransform2d()-position.pos
        val robotRelPosErr = posErr.vector().rotate((-position.pos.angle).cast(rad))
        val headingErr = posErr.angle

        val vErr = Transform2D(
            vec2(sample.vx.m/s,sample.vy.m/s)
                .rotate((-position.pos.angle).cast(rad)),
            sample.omega.rad/s
        ) - position.vel.exp()

        val acc = Transform2D(vec2(sample.ax.m/s/s,sample.ay.m/s/s).rotate((-position.pos.angle).cast(
            rad)),sample.alpha.rad/s/s)


        val p = Transform2D(robotRelPosErr*coeff.p/s/s,headingErr*angCoeff.p/s/s)
        val d = Transform2D(vErr.vector()*coeff.d/s,vErr.angle*angCoeff.d/s)

        val targetAcc = acc*a + p + d

        return SwerveAccelerationController.Target(targetAcc)
    }
}

fun choreoAccControllerLoop(
    choreoController: ChoreoAccController, swerveLoop: ControlLoopContext<*, *, SwerveAccelerationController.Target, *, SwerveAccelerationController>
) = ControlLoopContext(
    LoopTimes.CHOREO,
    choreoController,
    { io: SigmaIO ->
        RobotMovementState(io.position(), io.velocity())
    },
    { u: SwerveAccelerationController.Target, io: SigmaIO ->
        swerveLoop.lock.withLock {
            swerveLoop.controller.target = u
            swerveLoop.controller.logPosition = choreoController.position.pos
        }
    },
    {
        if(LOGGING.RERUN_CHOREO) {
            it.trajectoryLogger.logTraj(it.target,TrajectoryLogger.Mode.VEL_ARROW)
            it.trajectoryLogger.highlightSample("closestSample",it.sample)
            scalar("choreo/vDes/vx",it.sample.vx)
            scalar("choreo/vDes/vy",it.sample.vy)
            scalar("choreo/vDes/vo",it.sample.omega)
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
