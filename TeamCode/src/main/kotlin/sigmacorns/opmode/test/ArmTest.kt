package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.cast
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ArmTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io, DiffyOutputPose(0.rad,0.rad), DiffyOutputPose(0.m,0.m), )

        robot.slides.disabled = true
        robot.intake.disabled = true

        waitForStart()

        var lastT = io.time()

        while (opModeIsActive()) {
            val t = io.time()
            val dt = t-lastT
            lastT = t

            robot.arm.t = robot.arm.kinematics.underInverse(robot.arm.t.let {
                 DiffyOutputPose(
                    it.axis1 - gamepad1.left_stick_y*dt*0.1.rad/s,
                    it.axis2 - gamepad1.right_stick_y*dt*0.1.rad/s
                 ) } ) {
                    DiffyInputPose(
                        Limits.ARM_SERVO_1.apply(it.axis1.cast(rad)),
                        Limits.ARM_SERVO_2.apply(it.axis2.cast(rad))
                    )
                }

            robot.update(dt.value)
        }
    }
}