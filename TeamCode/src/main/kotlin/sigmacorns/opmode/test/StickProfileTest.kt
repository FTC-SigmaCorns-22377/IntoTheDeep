package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.SigmaIO
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class StickProfileTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("RIGHT_STICK_X",gamepad1.right_stick_x)
            telemetry.addData("ADJUSTED_RIGHT_STICK_X",Tuning.stickProfile(gamepad1.right_stick_x))
            telemetry.update()
        }
    }
}