package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.SigmaIO
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ColorTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        waitForStart()

        while (opModeIsActive()) {
            io.updateColorDist()

            telemetry.addData("red",io.red())
            telemetry.addData("green",io.green())
            telemetry.addData("blue",io.blue())
            telemetry.addData("alpha",io.alpha())
            telemetry.addData("distance",io.distance())
            telemetry.update()
        }
    }
}