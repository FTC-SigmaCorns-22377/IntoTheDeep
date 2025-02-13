package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.core.unaryMinus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning

@TeleOp
class ManualTeleop: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m), Tuning.IntakePosition.OVER)

        val g1 = GamepadEx(gamepad1)
        val g2 = GamepadEx(gamepad2)

        val maxSpeed = robot.drivebase.motor.topSpeed(1.0)*robot.drivebase.radius
        val maxAngSpeed = maxSpeed/(robot.drivebase.length/2.0+robot.drivebase.width/2.0)

        var slidesTarget = robot.slides.t

        waitForStart()

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t

            val v = vec2(-gamepad1.left_stick_y,gamepad1.left_stick_x)
            robot.mecanum.t = Transform2D(v*maxSpeed, -maxAngSpeed*gamepad1.right_stick_x)

            val activePower = gamepad1.left_trigger - gamepad1.right_trigger
            robot.active.updatePort(activePower*Tuning.ACTIVE_POWER)
            robot.intake.t = if(g1.rightBumper.isToggled) Tuning.IntakePosition.BACK else Tuning.IntakePosition.OVER
            robot.claw.updatePort(if(g1.a.isToggled) Tuning.CLAW_CLOSED else Tuning.CLAW_OPEN)

            slidesTarget = DiffyOutputPose(
                slidesTarget.axis1 - gamepad2.left_stick_y*dt*20.cm/ s,
                slidesTarget.axis2 - gamepad2.right_stick_y*dt*20.cm/ s,
            )

            slidesTarget.axis1 = Limits.EXTENSION.apply(slidesTarget.axis1.cast(m))
            slidesTarget.axis2 = Limits.LIFT.apply(slidesTarget.axis2.cast(m))

            val armPower = gamepad1.dpad_up.toInt() - gamepad1.dpad_down.toInt()
            val wristPower = gamepad1.dpad_right.toInt() - gamepad1.dpad_left.toInt()

            robot.arm.t = robot.arm.kinematics.underInverse(robot.arm.t.let {
                DiffyOutputPose(
                    it.axis1 - armPower*dt*0.5.rad/s,
                    it.axis2 - wristPower*dt*0.5.rad/s
                ) } ) {
                DiffyInputPose(
                    Limits.ARM_SERVO_1.apply(it.axis1.cast(rad)),
                    Limits.ARM_SERVO_2.apply(it.axis2.cast(rad))
                )
            }

            robot.slides.t = slidesTarget

            robot.update(dt.value)
            g1.periodic()
            g2.periodic()
            Scheduler.tick()
        }
    }
}

fun Boolean.toInt() = if(this) 1.0 else 0.0