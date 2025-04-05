package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.schedule
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose

@TeleOp
class TransferTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) = io.use {
        val robot = Robot(
            io,
            DiffyOutputPose(0.rad,0.rad),
            DiffyOutputPose(0.m,0.m),
        )

        val visualizer = RobotVisualizer(io)
        visualizer.init()

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

            println("RUNNING COMMANDS")
            for(cmd in Scheduler.cmds) {
                print("${cmd.name}(${cmd.status}), ")
            }
            println("--------------")

            robot.update(dt)
            visualizer.log()
        }
    }
}