package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import sigmacorns.common.Robot
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.choreoCommand
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.fastChoreoCommand
import sigmacorns.common.cmd.fastScore
import sigmacorns.common.cmd.score
import sigmacorns.common.cmd.timeout
import sigmacorns.common.cmd.wait
import sigmacorns.common.cmd.eject
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.getSample
import sigmacorns.common.cmd.intakeCommand
import sigmacorns.common.cmd.powerIntakeCommand
import sigmacorns.common.cmd.reset
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@Autonomous
class SpecimenAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(
            io,
            DiffyOutputPose(0.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            initPos = startPosFromTraj("push_specimen")
        )
        
        Scheduler.log = false
        Scheduler.reset()
        
        robot.update(0.0)

        specimenAuto(robot).schedule()
        
        waitForStart()

        
        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

//            println("RUNNING COMMANDS")
//            for(cmd in Scheduler.cmds) {
//                print("${cmd.name}(${cmd.status}), ")
//            }
//            println("--------------")

            robot.choreo.tickControlNode(dt.value)
            Scheduler.tick()
            robot.update(dt.value)
        }
    }
}

fun specimenAuto(robot: Robot) =
    series(
        clawCommand(robot,false),
        choreoCommand(robot,"push_specimen") + depoCommand(robot,Tuning.specimenWallPose),
        cycle(robot, true),
        cycle(robot, false)
    )   

fun cycle(robot: Robot, first: Boolean): Command {
    val scorePath = if(first) "score_first_specimen" else "score_specimen"
    return series(
        clawCommand(robot,true),
        fastChoreoCommand(robot,scorePath) + depoCommand(robot,ScorePosition.HIGH_SPECIMEN),
        fastScore(robot,ScorePosition.HIGH_SPECIMEN),
        fastChoreoCommand(robot, "grab_specimen") + depoCommand(robot,Tuning.specimenWallPose),
    ) 
}

fun betterSpecimenAuto(robot:Robot) =
    series(

    )

fun dropoff(robot: Robot): Command{
    return series(
        clawCommand(robot, true),
        choreoCommand(robot, "Preload") + depoCommand(robot, ScorePosition.HIGH_SPECIMEN),
        score(robot, ScorePosition.HIGH_SPECIMEN),
        choreoCommand(robot, "Post Preload") + reset(robot) + (wait(500.ms) then extendCommand(robot, 0.1.m)),
        getSample(robot),
        choreoCommand(robot, "Dropoff 1"),
        eject(robot),
        choreoCommand(robot, "Dropoff 2"),
        getSample(robot),
        choreoCommand(robot, "Dropoff 3"),
        eject(robot),
        choreoCommand(robot, "Dropoff 4"),
        getSample(robot),
        choreoCommand(robot, "Dropoff 5"),
        eject(robot)
    )
}

operator fun Command.times(n: Int) = series(*Array(n) { this })

fun startPosFromTraj(name: String) = Choreo.loadTrajectory<SwerveSample>(name).get().initialPose.toTransform2d()