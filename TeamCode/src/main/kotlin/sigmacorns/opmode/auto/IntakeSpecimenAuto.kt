package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inches
import sigmacorns.common.Robot
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.choreoCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.eject
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.fastScore
import sigmacorns.common.cmd.getSample
import sigmacorns.common.cmd.wait
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode


@Autonomous
class IntakeSpecimenAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(
            io,
            DiffyOutputPose(0.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            initPos = startPosFromTraj("Preload")
        )

        Scheduler.log = false
        Scheduler.reset()

        robot.update(0.0)

        intakeSpecimenAuto(robot).schedule()

        robot.claw.updatePort(Tuning.CLAW_CLOSED)
        robot.claw.node.tickControlNode(0.0)

        waitForStart()

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

fun intakeSpecimenAuto(robot: Robot): Command {
    return series(
        choreoCommand(robot, "Preload") + depoCommand(robot, ScorePosition.HIGH_SPECIMEN),
        fastScore(robot, ScorePosition.HIGH_SPECIMEN),
        parallel(
            choreoCommand(robot, "Post Preload"),
            depoCommand(robot, Tuning.specimenWallPose),
            wait(700.ms) then (extendCommand(robot, 11.inches) + getSample(robot)),
        ),
        choreoCommand(robot, "Dropoff 1"),
        eject(robot),
        choreoCommand(robot, "Dropoff 2") + extendCommand(robot, 12.inches),
        getSample(robot),
        choreoCommand(robot, "Dropoff 3"),
        eject(robot)
    )
}