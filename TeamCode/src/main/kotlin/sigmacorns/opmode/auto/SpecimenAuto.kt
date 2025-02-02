package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import sigmacorns.common.Robot
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.intakeCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@Autonomous
class SpecimenAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, DiffyOutputPose(0.rad, 0.rad), DiffyOutputPose(0.m,0.m))

        val cmd = intakeCommand(robot,10.cm) then liftCommand(robot,5.cm)

        val cmdExample2 =
            (
                depoCommand(robot,Tuning.specimenLowPose) + cmd { instant { robot.claw.updatePort(0.0) } }
            ) then cmd { instant { println("WHEEE") } }

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