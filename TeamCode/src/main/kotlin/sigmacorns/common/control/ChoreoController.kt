package sigmacorns.common.control

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.cm
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
import kotlin.math.absoluteValue

data class RobotMovementState(val pos: Transform2D, val vel: Twist2D)

class ChoreoController(
    val coeff: PIDCoefficients,
    val angCoeff: PIDCoefficients,
    val drivebaseSize: Vector2,
    val rerunDownscale: Int = 1
): Controller<RobotMovementState, Transform2D, Trajectory<SwerveSample>>() {
    override var output: Transform2D = Transform2D(0.m/s,0.m/s,0.rad/s)
    override var position: RobotMovementState = RobotMovementState(Transform2D(0.m,0.m,0.rad),
        Twist2D(0.m/s,0.m/s,0.rad/s)
    )
    override lateinit var target: Trajectory<SwerveSample>

    val trajectoryLogger = TrajectoryLogger(drivebaseSize.x.cast(m),drivebaseSize.y.cast(m))

    private var lastTraj: Trajectory<SwerveSample>? = null

    var t = 0.0

    lateinit var sample: SwerveSample

    val xController = PIDController(coeff)
    val yController = PIDController(coeff)
    val angController = PIDController(angCoeff)

    override fun copy() = TODO()

    override fun update(deltaTime: Double): Transform2D {
        if(lastTraj == target) t += deltaTime else t = 0.0
        lastTraj = target

//        sample = target.getClosestSample(position.pos.toPose2d())
//        sample = target.sampleAt(sample.t + 1.ms.value)

        sample = target.sampleAt(t)
        val endSample = target.finalSample.pose

//        if((sample.pose.toTransform2d() - endSample.toTransform2d()).vector().magnitude() < 5.cm)
//            sample = target.finalSample

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

//        val res =
//            Transform2D(robotRelPosErr*coeff.p, headingErr*angCoeff.p) +
//            (velBase + Twist2D(velErr.vector()*coeff.d,velErr.dAngle*angCoeff.d)).exp() +
//            Twist2D(acc.vector()*coeff.i,acc.dAngle*angCoeff.i).exp()

        val res = Transform2D(
            xController.updateStateless(deltaTime,0.0,robotRelPosErr.x.value).m/s,
            yController.updateStateless(deltaTime,0.0,robotRelPosErr.y.value).m/s,
            angController.updateStateless(deltaTime,0.0,headingErr.value).rad/s
        )

        return res.let { Transform2D(it.x,-it.y,it.angle) }
    }
}

fun choreoControllerLoop(
    choreoController: ChoreoController,
    robot: Robot
) = choreoController.toControlLoop("choreo",robot.io,{
    RobotMovementState(io.position(),io.velocity())
},{
    robot.mecanum.t = it
},{ x,t ->
    val f = t.finalSample
    (x.pos-f.pose.toTransform2d()).let {
        it.vector().magnitude() < Tuning.choreoPosThresh
        it.angle.normalizeRadian().map { it.absoluteValue } < Tuning.choreoAngThresh
    } && (x.vel.vector()-vec2(f.vx.m/s,f.vy.m/s)).magnitude() < Tuning.choreoVelThresh
            && (x.vel.dAngle-f.omega.rad/s).normalizeRadian().map { it.absoluteValue } < Tuning.choreoAngVelThresh
})

private fun Transform2D.toPose2d(): Pose2D = Pose2D(DistanceUnit.METER, x.value, y.value, AngleUnit.RADIANS, angle.value)
private fun Pose2D.toTransform2d(): Transform2D = Transform2D(
    getX(DistanceUnit.METER).m,
    getY(DistanceUnit.METER).m,
    getHeading(AngleUnit.RADIANS).rad
)