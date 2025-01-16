package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.util.Clock

@TeleOp
class ServoCycle: LinearOpMode() {
    var targetPos = 0.5
    val speed = 0.5
    override fun runOpMode() {
        waitForStart()

        val servo = hardwareMap.get(Servo::class.java,"clawServo")
        var lastT = Clock.seconds
        while (opModeIsActive()) {
            val t = Clock.seconds
            val dt = t-lastT
            lastT = t

            targetPos -= gamepad1.left_stick_y * dt *speed
            targetPos = targetPos.clampMagnitude(1.0)
            servo.position = targetPos

            telemetry.addData("pos",targetPos)
            telemetry.update()
        }
    }
}