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
class SlidesManualTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {

        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m),)

        robot.slides.disabled = true
        robot.arm.disabled = true
        robot.mecanum.disabled = true

        waitForStart()

        var lastT = io.time()

        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t

            io.motor1 = if(gamepad1.a) 0.5
            else if(gamepad1.b) -0.5
            else 0.0

            io.motor2 = if(gamepad1.x) 0.5
            else if(gamepad1.y) -0.5
            else 0.0
        }
    }
}