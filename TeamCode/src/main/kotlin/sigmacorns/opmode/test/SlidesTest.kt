package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.currency.SCHF
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.tick
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.resetDepo
import sigmacorns.common.cmd.resetIntake
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.DynamicPIDCoefficients1
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

            robot.slidesController.bypassAxis1 = (-gamepad1.left_stick_y.toDouble()).takeIf { gamepad1.left_stick_button }
            robot.slidesController.bypassAxis2 = (-gamepad1.right_stick_y.toDouble()).takeIf { gamepad1.right_stick_button }

            robot.slides.t = target

            if(gamepad1.a) extendCommand(robot,50.cm).schedule()
            if(gamepad1.b) liftCommand(robot,50.cm).schedule()
            if(gamepad1.x) (resetDepo(robot) + resetIntake(robot)).schedule()
            if(gamepad1.y) robot.io.resetSlideMotors(0.tick,0.tick)

            robot.slidesController.axis1PIDCoefficients = DynamicPIDCoefficients1.coeff()
            robot.slidesController.axis2PIDCoefficients = DynamicPIDCoefficients1.coeff()

            robot.update(dt.value)

            telemetry.addData("m1", robot.io.motor1Pos())
            telemetry.addData("m2", robot.io.motor2Pos())

            telemetry.addData("m1Pow", robot.io.motor1)
            telemetry.addData("m2Pow", robot.io.motor2)
            telemetry.addData("extend=",robot.slidesController.position.axis1)
            telemetry.addData("lift=",robot.slidesController.position.axis2)
            telemetry.addData("intakeTouch=",robot.io.intakeLimitTriggered())
            telemetry.addData("liftTouch=",robot.io.liftLimitTriggered())
            telemetry.update()

            Scheduler.tick()
        }
    }
}
