package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.RobotIO

@TeleOp
class ZeroScoringServos : LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)

        io.diffyServos?.forEach { it.position = 0.0 }

        io.clawServo?.position = 0.0
    }
}