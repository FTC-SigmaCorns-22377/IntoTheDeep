package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
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
import net.unnamedrobotics.lib.math2.inches
import sigmacorns.common.Robot
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.cmd.choreoCommand
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.getSample
import sigmacorns.common.cmd.intakeCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.reset
import sigmacorns.common.cmd.score
import sigmacorns.common.control.ChoreoController
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.IntakePosition
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@Autonomous
class SampleAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(
            io,
            DiffyOutputPose(0.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            initPos = startPosFromTraj("Preload")
        )

        Scheduler.log = true
        Scheduler.reset()

        robot.update(0.0)

        waitForStart()

        sampleAuto(robot).schedule()

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            println("RUNNING COMMANDS")
            for(cmd in Scheduler.cmds) {
                print("${cmd.name}(${cmd.status}), ")
            }
            println("--------------")

            robot.choreo.tickControlNode(dt.value)
            Scheduler.tick()
            robot.update(dt.value)
        }
    }
}

fun sampleAuto(robot: Robot) =
    series(
        choreoCommand(robot, "Preload") + depoCommand(robot, Tuning.bucketHighPose),
        score(robot, ScorePosition.HIGH_BUCKET),
        reset(robot),
        robot.intake.follow(IntakePosition.BACK),
        choreoCommand(robot, "Pickup_1") + autoIntake(robot, 18.inches),
        choreoCommand(robot, "Score_1") + depoCommand(robot, Tuning.bucketHighPose),
        score(robot, ScorePosition.HIGH_BUCKET),
        reset(robot),
        robot.intake.follow(IntakePosition.BACK),
        choreoCommand(robot, "Pickup_2") + autoIntake(robot, 20.5.inches),
        choreoCommand(robot, "Score_2") + depoCommand(robot, Tuning.bucketHighPose),
        score(robot, ScorePosition.HIGH_BUCKET),
        reset(robot),
        robot.intake.follow(IntakePosition.BACK),
        choreoCommand(robot, "Pickup_3") + autoIntake(robot, 20.5.inches),
        choreoCommand(robot, "Score_3") + depoCommand(robot, Tuning.bucketHighPose),
        score(robot, ScorePosition.HIGH_BUCKET),
        reset(robot),
        choreoCommand(robot, "Ascent")
    )
