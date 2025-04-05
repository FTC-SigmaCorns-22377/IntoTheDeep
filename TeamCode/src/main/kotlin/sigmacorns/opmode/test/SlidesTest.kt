package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.currency.SCHF
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class SlidesTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {

        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m),)

        robot.arm.disabled = true
        robot.mecanum.disabled = true

        waitForStart()

        var lastT = io.time()


        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t

            var target = robot.slides.t

            target = DiffyOutputPose(
                target.axis1 - gamepad1.left_stick_y*dt*20.cm/s,
                target.axis2 - gamepad1.right_stick_y*dt*20.cm/s,
            )

            target.axis1 = Limits.EXTENSION.apply(target.axis1.cast(m))
            target.axis2 = Limits.LIFT.apply(target.axis2.cast(m))

            robot.slides.t = target

            if(gamepad1.a) extendCommand(robot,50.cm).schedule()
            if(gamepad1.b) liftCommand(robot,50.cm).schedule()

            robot.update(dt.value)
            Scheduler.tick()
        }
    }
}
