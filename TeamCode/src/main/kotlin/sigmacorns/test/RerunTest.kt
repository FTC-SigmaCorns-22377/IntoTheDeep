package sigmacorns.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.unnamedrobotics.lib.rerun.JNICallback
import net.unnamedrobotics.lib.rerun.RerunTest

@TeleOp
class RerunTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()

        val callback = object : JNICallback {
            override fun callback(string: String?) {
                println(string)
            }
        }

        println("BEFORE RUNNING TEST")
        RerunTest.test_rerun(callback)
        println("AFTER RUNNING TEST")

        while (opModeIsActive()) {
        }
    }
}