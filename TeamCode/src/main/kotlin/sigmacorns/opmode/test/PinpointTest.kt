package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class PinpointTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, )

        waitForStart()

        var lastT = io.time()

        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t
            robot.update(dt.value)

            val pos = io.position()
            telemetry.addData("x",pos.x)
            telemetry.addData("y",pos.y)
            telemetry.addData("angle",pos.angle)
        }
    }
}