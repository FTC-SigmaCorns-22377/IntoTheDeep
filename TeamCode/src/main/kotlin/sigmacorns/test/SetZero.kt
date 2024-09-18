package sigmacorns.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class SetZero: LinearOpMode() {
    override fun runOpMode() {
        (hardwareMap.get("zero") as Servo).position = 0.0
        waitForStart()
        while (opModeIsActive()) {
            (hardwareMap.get("zero") as Servo).position = 0.0
        }
    }
}