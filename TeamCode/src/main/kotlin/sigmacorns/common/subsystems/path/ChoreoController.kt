package sigmacorns.common.subsystems.path

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.rotate
import net.unnamedrobotics.lib.math2.vec2
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.common.LoopTimes
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.swerve.SwerveController

data class RobotMovementState(val pos: Transform2D, val vel: Twist2D)

class ChoreoController(
    val p: Double, val pAng: Double
): Controller<RobotMovementState, SwerveController.Target, Trajectory<SwerveSample>>() {
    override var output: SwerveController.Target = SwerveController.Target(Transform2D(0.m/s,0.m/s,0.rad/s),false)
    override lateinit var position: RobotMovementState
    override lateinit var target: Trajectory<SwerveSample>

    override fun copy() = ChoreoController(p, pAng)

    override fun update(deltaTime: Double): SwerveController.Target {
        val sample = target.getClosestSample(position.pos.toPose2d())

        val posErr = sample.pose.toTransform2d()-position.pos
        val robotRelPosErr = posErr.vector().rotate(position.pos.angle.cast(rad))
        val headingErr = posErr.angle

        val vel = Transform2D(
            vec2(sample.vx.m/s,sample.vy.m/s)
                .rotate((-position.pos.angle).cast(rad)),
            sample.omega.rad/s
        ) + Transform2D(robotRelPosErr*p/s,headingErr*pAng/s)

        vel.x.cast(m/s)
        vel.y.cast(m/s)
        vel.angle.cast(rad/s)

        return SwerveController.Target(vel,false)
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
        swerveLoop.target(u)
        swerveLoop.controller.logPosition = choreoController.position.pos
        swerveLoop.controller.logVelocity = choreoController.position.vel
    },
    name = "choreo"
)

fun Transform2D.toPose2d(): Pose2D = Pose2D(DistanceUnit.METER, x.value, y.value, AngleUnit.RADIANS, angle.value)
fun Pose2D.toTransform2d(): Transform2D = Transform2D(
    getX(DistanceUnit.METER).m,
    getY(DistanceUnit.METER).m,
    getHeading(AngleUnit.RADIANS).rad
)