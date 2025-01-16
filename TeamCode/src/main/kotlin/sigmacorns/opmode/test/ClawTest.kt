package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.Robot
import sigmacorns.common.io.RobotIO

@TeleOp
class ClawTest: LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)
        val robot = Robot(io)


        var pitch = 0.rad
        var roll = 0.rad

        waitForStart()

        robot.launchIOLoop()

        robot.inputLoop { dt ->
            pitch = pitch.map { it + dt.value*gamepad1.left_stick_y  }
            roll = roll.map { it + dt.value*gamepad1.left_stick_x  }
        }
    }
}