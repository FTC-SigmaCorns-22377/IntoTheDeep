package sigmacorns.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
class MotorTest: LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.get(DcMotor::class.java, "m1")

        waitForStart()

        while (opModeIsActive()) {
            motor.power = gamepad1.left_stick_y.toDouble()
        }
    }
}