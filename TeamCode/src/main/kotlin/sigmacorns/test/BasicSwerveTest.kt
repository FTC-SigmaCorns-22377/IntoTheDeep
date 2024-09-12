package sigmacorns.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
class BasicSwerveTest: LinearOpMode() {
    override fun runOpMode() {
        val drive1 = hardwareMap.get(DcMotor::class.java, "m1")
        val drive2 = hardwareMap.get(DcMotor::class.java, "m2")
        val drive3 = hardwareMap.get(DcMotor::class.java, "m3")
        val drive4 = hardwareMap.get(DcMotor::class.java, "m4")

        val turn1 = hardwareMap.get(CRServo::class.java, "t1")
        val turn2 = hardwareMap.get(CRServo::class.java, "t2")
        val turn3 = hardwareMap.get(CRServo::class.java, "t3")
        val turn4 = hardwareMap.get(CRServo::class.java, "t4")

        waitForStart()

        while (opModeIsActive()) {
            drive1.power = gamepad1.left_stick_y.toDouble()
            drive2.power = gamepad1.left_stick_y.toDouble()
            drive3.power = gamepad1.left_stick_y.toDouble()
            drive4.power = gamepad1.left_stick_y.toDouble()

            turn1.power = gamepad1.left_stick_x.toDouble()
            turn2.power = gamepad1.left_stick_x.toDouble()
            turn3.power = gamepad1.left_stick_x.toDouble()
            turn4.power = gamepad1.left_stick_x.toDouble()
        }
    }
}
