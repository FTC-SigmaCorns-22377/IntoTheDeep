package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.mapRanges
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.IntakeAngleKinematics
import sigmacorns.constants.Limits
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class IntakeKinematicsTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val kinematics = IntakeAngleKinematics

        var target: Expression = 0.rad
        var wasButtonPressed = false

        waitForStart()


        while (opModeIsActive()) {
            val angle = kinematics.inverse(target.cast(rad))

            if(gamepad1.a && !wasButtonPressed) target += 5.degrees
            if(gamepad1.b && !wasButtonPressed) target -= 5.degrees
            if(gamepad1.y && !wasButtonPressed) target += 1.degrees
            if(gamepad1.x && !wasButtonPressed) target -= 1.degrees

            wasButtonPressed = gamepad1.let { it.a || it.b || it.x || it.y }
            val angle1 = Limits.INTAKE_SERVO_1.apply(angle)
            val angle2 = Limits.INTAKE_SERVO_2.apply(angle)

            io.intakeL = mapRanges(Limits.INTAKE_SERVO_1.let { it.min.value..it.max.value },0.0..1.0)(angle1.value).toDouble()
            io.intakeR = mapRanges(Limits.INTAKE_SERVO_2.let { it.min.value..it.max.value },0.0..1.0)(angle2.value).toDouble()
        }
    }
}