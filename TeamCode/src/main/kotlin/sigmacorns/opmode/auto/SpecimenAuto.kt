package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import sigmacorns.common.Robot
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.armCommand
import sigmacorns.common.cmd.choreoCommand
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.fastChoreoCommand
import sigmacorns.common.cmd.fastScore
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
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
        
        Scheduler.log = false
        Scheduler.reset()
        
        robot.update(0.0)

        robot.claw.updatePort(Tuning.CLAW_CLOSED)
        robot.claw.node.tickControlNode(0.0)

        io.tilt1 = TiltPositions.AUTO.x
        io.tilt2 = TiltPositions.AUTO.x

        pushSpecimenAuto(robot).schedule()
        
        waitForStart()

        var loop = true
        var requestedEndTime = 0.s
        
        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            robot.choreo.tickControlNode(dt.value)
            Scheduler.tick()
            robot.update(dt.value)
        }
    }
}

fun pushSpecimenAuto(robot: Robot) =
    series(
        choreoCommand(robot,"preload_specimen") + depoCommand(robot,ScorePosition.HIGH_SPECIMEN),
        fastScore(robot,ScorePosition.HIGH_SPECIMEN),
        choreoCommand(robot,"push_specimen") + depoCommand(robot,Tuning.specimenWallPose),
        cycle(robot, 1),
        cycle(robot, 2),
        cycle(robot,3),
        cycle(robot,4)
    )

fun cycle(robot: Robot, n: Int): Command {
    val scorePath = "score_specimen$n"
    return series(
        clawCommand(robot,true),
        fastChoreoCommand(robot,scorePath) + depoCommand(robot,ScorePosition.HIGH_SPECIMEN),
        fastScore(robot,ScorePosition.HIGH_SPECIMEN).let { if(n==4) it then armCommand(robot,0.rad,0.rad) else it },
        fastChoreoCommand(robot, "grab_specimen") + depoCommand(robot,Tuning.specimenWallPose.let { if(n==4) LiftPose(it.lift,0.rad,0.rad) else it }),
    ) 
}

fun dropoff(robot: Robot): Command {
    return series(

    )
}

operator fun Command.times(n: Int) = series(*Array(n) { this })

fun startPosFromTraj(name: String) = Choreo.loadTrajectory<SwerveSample>(name).get().initialPose.toTransform2d()