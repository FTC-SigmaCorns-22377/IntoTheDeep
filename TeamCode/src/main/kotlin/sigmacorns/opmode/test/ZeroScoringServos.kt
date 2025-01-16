package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.RobotIO

@TeleOp
class ZeroScoringServos : LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)

        waitForStart()

        io.diffyServos.forEach { it.position = 0.0 }

        io.clawServo?.position = 0.0

        while (opModeIsActive()) {
            if (gamepad1.a) io.diffyServos[0].position = 0.0
            if (gamepad1.b) io.diffyServos[1].position = 1.0
            if (gamepad1.x) io.diffyServos[0].position = 1.0
            if (gamepad1.y) io.diffyServos[1].position = 0.0

            if (gamepad1.dpad_up) {
                io.diffyServos[0].position = 90.0/355.0
                io.diffyServos[1].position = 1.0 - 90.0/355.0
            }

            if(gamepad1.dpad_down) {
                io.diffyServos.map { it.position = 0.5 }
            }
        }
    }
}