package sigmacorns

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.unnamedrobotics.lib.gamepad.GamepadEx
import sigmacorns.common.Robot
import sigmacorns.common.io.RobotIO

@TeleOp
class Tele: LinearOpMode() {
    lateinit var robot: Robot

    val gm1 = GamepadEx(gamepad1)
    val gm2 = GamepadEx(gamepad2)

    override fun runOpMode() {
        robot = Robot(RobotIO(hardwareMap))

        waitForStart()

        robot.launchIOLoop()

        while (opModeIsActive()) {
//            gm1.rightStick.
        }
    }
}