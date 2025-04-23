package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.transmute
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.rerun
import org.joml.Quaterniond
import sigmacorns.common.Robot
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.armCommand
import sigmacorns.common.cmd.choreoCommand
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.fastChoreoCommand
import sigmacorns.common.cmd.fastScore
import sigmacorns.common.cmd.timeout
import sigmacorns.common.control.TrajectoryLogger
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Physical
import sigmacorns.constants.TiltPositions
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode
import kotlin.math.min


@Autonomous
class SpecimenAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(
            io,
            DiffyOutputPose(90.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            initPos = startPosFromTraj("preload_specimen")
        )

        val vSens = hardwareMap.get<VoltageSensor>(VoltageSensor::class.java, "Control Hub")
        val v = vSens.voltage
        robot.mecanum.baseScalar = min(1.0,12.7/v)
        
//        Scheduler.log = false
        Scheduler.reset()
        
        robot.update(0.0)

        robot.claw = Tuning.CLAW_CLOSED

        io.tilt1 = TiltPositions.STRAIGHT.x
        io.tilt2 = TiltPositions.STRAIGHT.x

        pushSpecimenAuto(robot).schedule()
        
        waitForStart()

        var loop = true
        var requestedEndTime = 0.s
        
        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            Scheduler.tick()
            robot.choreo.tickControlNode(dt.value)
            robot.update(dt.value)

            rerun(robot.io.rerunConnection) {
                if(robot.choreo.t.isPresent) {
                    robot.trajLogger.logTraj(robot.choreo.t.get(), TrajectoryLogger.Mode.POINTS)
                    robot.trajLogger.highlightSample("bot",robot.choreoController.sample)
                    val s = robot.choreoController.position.pos
                    log("pos") { Boxes3D(
                        halfSizes = listOf( vec3(Physical.DRIVEBASE_SIZE.x/2.0,Physical.DRIVEBASE_SIZE.y/2.0,0.m) ),
                        centers = listOf(vec3(s.x,s.y,0.m)),
                        rotations = listOf(Quaterniond().setAngleAxis(s.angle.value,0.0,0.0,1.0))
                    ) }
//                    viz.logTwist(robot.choreoController.output.log().let { Twist2D(it.dx.transmute(m),it.dy.transmute(m),it.dAngle.transmute(
//                        rad)) },"drive",10,20,origin = s.vector().withZ(0.m))
                }
//                viz.logScalars()
            }
        }
    }
}

fun pushSpecimenAuto(robot: Robot) =
    series(
        (choreoCommand(robot,"preload_specimen") + depoCommand(robot,ScorePosition.HIGH_SPECIMEN)).timeout(1.5.s),
        fastScore(robot,ScorePosition.HIGH_SPECIMEN).timeout(1.s),
        (choreoCommand(robot,"push_specimen") + depoCommand(robot,Tuning.specimenWallPose)).timeout(10.s),
        cycle(robot, 1),
        cycle(robot, 2),
        cycle(robot,3),
        cycle(robot,4)
    )

fun cycle(robot: Robot, n: Int): Command {
    val scorePath = "score_specimen$n"
    return series(
        clawCommand(robot,true),
        (fastChoreoCommand(robot,scorePath) + depoCommand(robot,ScorePosition.HIGH_SPECIMEN)).timeout(2.5.s),
        fastScore(robot,ScorePosition.HIGH_SPECIMEN).let {
            if(n==4) it then armCommand(robot,0.rad,0.rad) else it
        }.timeout(1.s),
        (fastChoreoCommand(robot, "grab_specimen") + depoCommand(robot,Tuning.specimenWallPose.let {
            if(n==4) LiftPose(it.lift,0.rad,0.rad) else it
        })).timeout(3.s),
    ) 
}

fun dropoff(robot: Robot): Command {
    return series(

    )
}

operator fun Command.times(n: Int) = series(*Array(n) { this })

fun startPosFromTraj(name: String) = Choreo.loadTrajectory<SwerveSample>(name).get().initialPose.toTransform2d()