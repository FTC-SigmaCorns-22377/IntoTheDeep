package sigmacorns.opmode.test

import com.acmerobotics.dashboard.DashboardCore
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.ms
import net.unnamedrobotics.lib.server.dashboard.Dashboard
import sigmacorns.common.Robot
import sigmacorns.common.io.SigmaIO
import sigmacorns.constants.Color
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ColorTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        waitForStart()

        while (opModeIsActive()) {
            val robot = Robot(io)
            io.updateSensors()

            val tel = FtcDashboard.getInstance().telemetry

            fun addData(caption: String, msg: Any?) {
                tel.addData(caption,msg)
                telemetry.addData(caption,msg)
            }

            addData("red",io.red())
            addData("green",io.green())
            addData("blue",io.blue())
            addData("alpha",io.alpha())
            addData("distance",io.distance())
            addData("sensed color", robot.color())

            telemetry.update()
            tel.update()
        }
    }
}