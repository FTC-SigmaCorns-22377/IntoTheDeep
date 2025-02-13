package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math.clamp
import sigmacorns.common.io.SigmaIO
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ClawTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {

        waitForStart()

        var lastT = io.time()
        var curTarget = Tuning.CLAW_OPEN

        var wasPressed = false

        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t

            curTarget += gamepad1.left_stick_x*dt.value*0.5
            if (gamepad1.a && !wasPressed) curTarget += 0.05
            if (gamepad1.b && !wasPressed) curTarget -= 0.05

            wasPressed = gamepad1.a || gamepad1.b
            curTarget = clamp(0.0,curTarget,1.0)

            telemetry.addData("pos",curTarget)
            telemetry.update()

            io.claw = curTarget
        }
    }
}