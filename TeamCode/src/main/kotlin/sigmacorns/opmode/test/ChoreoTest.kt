package sigmacorns.opmode.test

import android.graphics.Path.Op
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.transmute
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.rerun
import org.joml.Quaterniond
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.control.TrajectoryLogger
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.constants.DynamicPIDCoefficients1
import sigmacorns.constants.Physical
import sigmacorns.opmode.SIM
import sigmacorns.opmode.SimOrHardwareOpMode
import java.io.File
import java.util.Optional

@TeleOp
class ChoreoTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        io.rerunConnection
        if(SIM) {
            io.rerunConnection.setTimeSeconds("sim",0.s)
            val dir = System.getProperty("user.dir")
            io.rerunConnection.field("$dir/src/test/resources/field_image.png")
            Choreo::class.java.getDeclaredField("CHOREO_DIR").let {
                it.isAccessible = true
                it.set(null, File(dir!!).parentFile!!.resolve("paths/choreo") )
            }
        }

        var trajName = "straight back"


        val viz = RobotVisualizer(io)

        while (opModeInInit()) {
            if(gamepad1.a) trajName = "straight back"
            if(gamepad1.b) trajName = "HOPE"
            telemetry.addData("traj=",trajName)
            telemetry.update()
        }

        waitForStart()

        val traj = (Choreo::loadTrajectory)(trajName).get() as Trajectory<SwerveSample>
        val robot = Robot(io, initPos = traj.initialPose.toTransform2d())

        robot.choreo.t = Optional.of(traj)

        robot.update(0.0)

        viz.init()

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            robot.choreoController.coeff = DynamicPIDCoefficients1.coeff()
            robot.choreoController.angCoeff = DynamicPIDCoefficients1.coeff()
            robot.choreo.tickControlNode(dt.value)
            robot.update(dt.value)

//            if(SIM) sleep(50)
            val pos = robot.choreo.x.pos

            telemetry.addData("x",pos.x)
            telemetry.addData("y",pos.y)
            telemetry.addData("angle",pos.angle)
            telemetry.update()

            rerun(robot.io.rerunConnection) {
                if(robot.choreo.t.isPresent) {
                    robot.trajLogger.logTraj(robot.choreo.t.get(),TrajectoryLogger.Mode.POINTS)
                    robot.trajLogger.highlightSample("bot",robot.choreoController.sample)
                    val s = robot.choreoController.position.pos
                    log("pos") { Boxes3D(
                        halfSizes = listOf( vec3(Physical.DRIVEBASE_SIZE.x/2.0,Physical.DRIVEBASE_SIZE.y/2.0,0.m) ),
                        centers = listOf(vec3(s.x,s.y,0.m)),
                        rotations = listOf(Quaterniond().setAngleAxis(s.angle.value,0.0,0.0,1.0))
                    ) }
                    viz.logTwist(robot.choreoController.output.log().let { Twist2D(it.dx.transmute(m),it.dy.transmute(m),it.dAngle.transmute(
                        rad)) },"drive",10,20,origin = s.vector().withZ(0.m))
                }
//                viz.logScalars()
            }
        }
    }
}