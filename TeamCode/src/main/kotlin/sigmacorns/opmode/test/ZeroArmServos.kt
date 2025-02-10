package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math.clamp
import net.unnamedrobotics.lib.math2.mapRanges
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ZeroArmServos: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val pos = mutableListOf(0.0,0.0)
        var cur = 0
        var bumperJustPressed = false

        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m),)

        waitForStart()

        var wasButtonPressed = false

        while (opModeIsActive()) {
            if(gamepad1.left_bumper && !bumperJustPressed) {
                cur = 1-cur
                bumperJustPressed = true
                continue
            }
            bumperJustPressed = gamepad1.left_bumper

            if(gamepad1.right_bumper) pos[cur] = 0.0
            if(gamepad1.a && !wasButtonPressed) pos[cur] += 0.05
            if(gamepad1.b && !wasButtonPressed) pos[cur] -= 0.05
            if(gamepad1.y && !wasButtonPressed) pos[cur] += 0.01
            if(gamepad1.x && !wasButtonPressed) pos[cur] -= 0.01

            if(gamepad1.dpad_up && !wasButtonPressed) {
                pos[0] += 0.03
                pos[1] += 0.03
            }

            if(gamepad1.dpad_down && !wasButtonPressed) {
                pos[0] -= 0.03
                pos[1] -= 0.03
            }

            if(gamepad1.dpad_right && !wasButtonPressed) {
                pos[0] += 0.03
                pos[1] -= 0.03
            }

            if(gamepad1.dpad_left && !wasButtonPressed) {
                pos[0] -= 0.03
                pos[1] += 0.03
            }

            pos[cur] = clamp(0.0,pos[cur],1.0)

            wasButtonPressed = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right

            telemetry.addData("cur",cur)
            telemetry.addData("pos1",pos[0])
            telemetry.addData("pos2",pos[1])

            val estimted = robot.arm.kinematics.forward(DiffyInputPose(
                mapRanges(0.0..1.0,Limits.ARM_SERVO_1.let { it.min.value..it.max.value })(pos[0]).rad,
                mapRanges(0.0..1.0,Limits.ARM_SERVO_2.let { it.min.value..it.max.value })(pos[1]).rad,
            ))
            val e = robot.arm.kinematics.inverse(estimted)
            telemetry.addData("estimatedArm=",estimted.axis1)
            telemetry.addData("estimatedWrist=",estimted.axis2)
            telemetry.addData("estimatedGlobalWrist=",estimted.axis1+estimted.axis2)
            telemetry.addData("e1",e.axis1)
            telemetry.addData("e2",e.axis2)

            // good: 0.09, 0.0
            // arm = -2.22, wrist= 0.77

            // good transfer: 1, 0.87
            // arm = 2.375, wrist = 0.877
            telemetry.update()

            io.armL = pos[0]
            io.armR = pos[1]
        }

    }
}