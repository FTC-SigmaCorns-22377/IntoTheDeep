package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.SigmaIO
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ClimbTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        waitForStart()

        while (opModeIsActive()) {
            io.motor1 = -(gamepad1.left_stick_y + gamepad1.right_stick_y).toDouble()
            io.motor2 = -(gamepad1.left_stick_y - gamepad1.right_stick_y).toDouble()
            io.motor3 = (io.motor1-io.motor2)/2.0
        }
    }
}
