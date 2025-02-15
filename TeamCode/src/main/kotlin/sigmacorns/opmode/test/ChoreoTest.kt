package sigmacorns.opmode.test

import android.graphics.Path.Op
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.control.TrajectoryLogger
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.opmode.SIM
import sigmacorns.opmode.SimOrHardwareOpMode
import java.io.File
import java.util.Optional

@TeleOp
class ChoreoTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        if(SIM) {
            Choreo::class.java.getDeclaredField("CHOREO_DIR").let {
                it.isAccessible = true
                it.set(null, File("C:\\Users\\chemi\\Documents\\choreo\\base"))
            }
            io.rerunConnection.setTimeSeconds("sim",0.s)
            val dir = System.getProperty("user.dir")
            io.rerunConnection.field("$dir/src/test/resources/field_image.png")
        }

        val traj = (Choreo::loadTrajectory)("push_specimen").get() as Trajectory<SwerveSample>
        val robot = Robot(io, initPos = traj.initialPose.toTransform2d())

//        val viz = RobotVisualizer(io)

        waitForStart()

        robot.choreo.t = Optional.of(traj)

        robot.update(0.0)


//        viz.init()

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            robot.choreo.tickControlNode(dt.value)
            robot.update(dt.value)

            if(SIM) sleep(50)
            val pos = robot.choreo.x.pos

            telemetry.addData("x",pos.x)
            telemetry.addData("y",pos.y)
            telemetry.addData("angle",pos.angle)
            telemetry.update()

//            rerun(robot.io.rerunConnection) {
//                robot.trajLogger.logTraj(robot.choreo.t,TrajectoryLogger.Mode.POINTS)
//            }
//            viz.log()
        }
    }
}