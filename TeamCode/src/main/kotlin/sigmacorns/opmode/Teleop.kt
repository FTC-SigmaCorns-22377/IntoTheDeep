package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.core.unaryMinus
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.armCommand
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.retract
import sigmacorns.common.cmd.transferCommand
import sigmacorns.common.cmd.wait
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning

@TeleOp
class Teleop: SimOrHardwareOpMode() {



    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(
            io,
            DiffyOutputPose(90.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            Robot.IntakePositions.OVER
        )

        val visualizer = RobotVisualizer(io)

        val g1 = GamepadEx(gamepad1)
        val g2 = GamepadEx(gamepad2)

        val maxSpeed = robot.drivebase.motor.topSpeed(1.0) * robot.drivebase.radius
        val maxAngSpeed = 0.8 * maxSpeed / (robot.drivebase.length / 2.0 + robot.drivebase.width / 2.0)


        Scheduler.reset()

        visualizer.init()

        waitForStart()

        robot.update(0.0)

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            val v = vec2(-gamepad1.left_stick_y, gamepad1.left_stick_x)
            robot.mecanum.t = Transform2D(v * maxSpeed, -maxAngSpeed * gamepad1.right_stick_x)

            val activePower =
                gamepad1.left_trigger - gamepad1.right_trigger + gamepad2.left_trigger - gamepad2.right_trigger
            robot.active.updatePort(activePower * Tuning.ACTIVE_POWER)

            // MANUAL SLIDES/ARM
            var slidesTarget = robot.slides.t
            slidesTarget = DiffyOutputPose(
                slidesTarget.axis1 - gamepad2.left_stick_y * dt * 20.cm / s,
                slidesTarget.axis2 - gamepad2.right_stick_y * dt * 20.cm / s,
            )

            slidesTarget.axis1 = Limits.EXTENSION.apply(slidesTarget.axis1.cast(m))
            slidesTarget.axis2 = Limits.LIFT.apply(slidesTarget.axis2.cast(m))

            val armPower = gamepad2.dpad_up.toInt() - gamepad2.dpad_down.toInt()
            val wristPower = gamepad2.dpad_right.toInt() - gamepad2.dpad_left.toInt()

            robot.arm.t = robot.arm.kinematics.underInverse(robot.arm.t.let {
                DiffyOutputPose(
                    it.axis1 - armPower * dt * 0.5.rad / s,
                    it.axis2 - wristPower * dt * 0.5.rad / s
                )
            }) {
                DiffyInputPose(
                    Limits.ARM_SERVO_1.apply(it.axis1.cast(rad)),
                    Limits.ARM_SERVO_2.apply(it.axis2.cast(rad))
                )
            }

            robot.slides.t = slidesTarget

            // END MANUAL POWER

            g1.periodic()
            g2.periodic()

            Scheduler.tick()

            robot.update(dt.value)
            visualizer.log()
        }
    }
}