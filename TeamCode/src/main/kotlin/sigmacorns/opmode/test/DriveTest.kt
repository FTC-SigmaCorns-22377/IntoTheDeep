package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.LastKnown
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class DriveTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m),)

        robot.intake.disabled = true
        robot.slides.disabled = true
        robot.arm.disabled = true

        val maxSpeed = robot.drivebase.motor.topSpeed(1.0)*robot.drivebase.radius
        val maxAngSpeed = maxSpeed/(robot.drivebase.length/2.0+robot.drivebase.width/2.0)
//
//            return drivebase.wheels.minOf {
//                tangentialSpeed/it.position.magnitude()
//            }
//        }

        waitForStart()

        var lastT = io.time()

        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t

            val v = vec2(-gamepad1.left_stick_y,gamepad1.left_stick_x)
            robot.mecanum.t = Transform2D(v*maxSpeed, -maxAngSpeed*gamepad1.right_stick_x)

            if(gamepad1.a) io.driveFL = 1.0
            else if(gamepad1.b) io.driveFR = 1.0
            else if(gamepad1.x) io.driveBL = 1.0
            else if(gamepad1.y) io.driveBR = 1.0
            else robot.update(dt.value)

        }
    }
}