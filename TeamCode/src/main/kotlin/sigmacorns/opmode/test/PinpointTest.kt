package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class PinpointTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, )

        robot.io.setPinPos(Transform2D(0.m,0.m,0.rad))
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
            telemetry.update()
        }
    }
}