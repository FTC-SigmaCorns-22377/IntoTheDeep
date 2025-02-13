package sigmacorns.opmode.test

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class TestTiltServos: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val pos = mutableListOf(0.0,0.0)
        var cur = 0
        var bumperJustPressed = false

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


            pos[cur] = clamp(0.0,pos[cur],1.0)

            wasButtonPressed = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right

            telemetry.addData("cur",cur)
            telemetry.addData("pos1",pos[0])
            telemetry.addData("pos2",pos[1])

            // good: 0.09, 0.0
            // arm = -2.22, wrist= 0.77

            // good transfer: 1, 0.87
            // arm = 2.375, wrist = 0.877
            telemetry.update()

            io.tilt1 = pos[0]
            io.tilt2 = pos[1]
        }
    }
}