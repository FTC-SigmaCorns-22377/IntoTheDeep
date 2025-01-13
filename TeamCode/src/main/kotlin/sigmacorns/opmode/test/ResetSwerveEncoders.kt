package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.common.io.RobotIO
import kotlin.math.PI

@TeleOp
class ResetSwerveEncoders: LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)

        io.turnEncoders.forEachIndexed { i,it -> println("encoder$i" + "${(it.voltage/3.3)* PI*2}") }
        telemetry.update()
        waitForStart()

        telemetry.update()
    }
}