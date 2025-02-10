package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.instant
import net.unnamedrobotics.lib.command.schedule
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.cmd.detectCommand
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.intakeCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.race
import sigmacorns.common.cmd.transferCommand
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Tuning

@TeleOp
class TransferTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) = io.use {
        val robot = Robot(
            io,
            DiffyOutputPose(0.rad,0.rad),
            DiffyOutputPose(0.m,0.m),
            Tuning.IntakePosition.OVER
        )

//        val visualizer = RobotVisualizer(io)
//        visualizer.init()

        waitForStart()

        robot.update(0.0)
        autoIntake(robot,0.5.m).schedule()
        robot.update(0.0)

        var lastT = io.time().value
        while (opModeIsActive()) {
            Scheduler.tick()
            val t = io.time().value
            val dt = t-lastT
            lastT = t

            robot.update(dt)
//            visualizer.log()
        }
    }
}