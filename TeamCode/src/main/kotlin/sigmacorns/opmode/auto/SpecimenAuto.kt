package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import sigmacorns.common.Robot
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.intakeCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.timeout
import sigmacorns.common.cmd.wait
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode
import sigmacorns.opmode.Teleop.ScorePositions

@Autonomous
class SpecimenAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, DiffyOutputPose(0.rad, 0.rad), DiffyOutputPose(0.m,0.m))

        val cmd = (((robot.mecanum.follow(Transform2D(1.m/s,0.m/s,0.rad/s)) + wait(700.ms))
                then (robot.mecanum.follow(Transform2D((0.5).m/s,0.m/s,0.rad/s)) + wait(600.ms))
                then (robot.mecanum.follow(Transform2D(0.m/s,0.m/s,0.rad/s)) + wait(100.ms)).timeout(4.s)
                then (depoCommand(robot, Tuning.asdf) + wait(100.ms)).timeout(4.s)
                then (depoCommand(robot, LiftPose((Tuning.asdf.lift-20.cm).cast(m), (Tuning.asdf.arm+30.degrees).cast(
            rad),Tuning.asdf.wrist)) + wait(100.ms)).timeout(4.s)
                then (cmd { instant {robot.claw.updatePort(Tuning.CLAW_OPEN)}} + wait(100.ms)).timeout(4.s)
                then (robot.mecanum.follow(Transform2D((-1).m/s,0.m/s,0.rad/s)) + wait(1200.ms))
                then (robot.mecanum.follow(Transform2D(0.m/s,1.m/s,0.rad/s)) + wait(1000.ms))
                then (robot.mecanum.follow(Transform2D(0.m/s,0.m/s,0.rad/s)) + wait(100.ms))).timeout(28.s)
                then (liftCommand(robot, 0.m))
                )

        io.claw = Tuning.CLAW_CLOSED

        waitForStart()

        robot.update(0.0)
        cmd.schedule()

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            Scheduler.tick()
            robot.update(dt.value)
        }
    }

}