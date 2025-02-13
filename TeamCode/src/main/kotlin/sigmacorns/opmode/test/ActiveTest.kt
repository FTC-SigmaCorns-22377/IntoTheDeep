package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.SigmaIO
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ActiveTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        waitForStart()

        while (opModeIsActive()) {
            io.intake = gamepad1.left_stick_y.toDouble()

        }
    }
}