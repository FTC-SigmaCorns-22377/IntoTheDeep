package sigmacorns.opmode.test

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.SigmaIO
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class IntakeManualTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        var pos = mutableListOf(Tuning.INTAKE_OVER_POS.first,Tuning.INTAKE_OVER_POS.second)
        var cur = 0
        var bumperJustPressed = false

//        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m),0.rad)

        waitForStart()

        var wasButtonPressed = false

        while (opModeIsActive()) {
            if(gamepad1.left_bumper && !bumperJustPressed) {
                cur = 1-cur
                bumperJustPressed = true
                continue
            }
            bumperJustPressed = gamepad1.left_bumper

//            if(gamepad1.right_bumper) pos[cur] = 0.0
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

            telemetry.update()

            io.intakeL = pos[0]
            io.intakeR = pos[1]
        }
    }
}
