package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.core.*
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Robot
import sigmacorns.common.Tuning
import sigmacorns.common.io.RobotIO
import sigmacorns.common.subsystems.swerve.SwerveController

@TeleOp
class SwerveTest: LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)
        val robot = Robot(io)

        gamepad1.type = Gamepad.Type.LOGITECH_F310
        val gm1 = GamepadEx(gamepad1)

        robot.addLoop(robot.swerveControlLoop)

        telemetry.clear()
        telemetry.addData("back", " -> selector")
        telemetry.addData("Y", " -> drive FL")
        telemetry.addData("B", " -> drive FR")
        telemetry.addData("X", " -> drive BL")
        telemetry.addData("A", " -> drive BR")

        telemetry.addData("^", " -> turn FL")
        telemetry.addData(">", " -> turn FR")
        telemetry.addData("<", " -> turn BL")
        telemetry.addData("v", " -> turn BR")
        telemetry.update()

        waitForStart()

        robot.launchIOLoop()

        var manual = false
        var wasStartPressed = false

        robot.inputLoop {
            gm1.periodic()

            runBlocking { robot.swerveControlLoop.withLock {
                robot.swerveControlLoop.controller.moduleControllers.forEach {
                    it.turnCoefficients = Tuning.SWERVE_MODULE_PID
                }
            } }

            if(gamepad1.start && !wasStartPressed) {
                manual = !manual
                robot.swerveControlLoop.write.set(!manual)
            }
            wasStartPressed = gamepad1.start

            val driveButtons = listOf(
                gm1.y,
                gm1.b,
                gm1.x,
                gm1.a
            )

            val turnButtons = listOf(
                gamepad1.dpad_up,
                gamepad1.dpad_right,
                gamepad1.dpad_left,
                gamepad1.dpad_down,
            )

            if(manual) {
                io.drives.zip(driveButtons)
                    .forEach { it.first.power = if(it.second.isPressed) 1.0 else 0.0 }

                io.turns.zip(turnButtons)
                    .forEach { it.first.power = if(it.second) 1.0 else 0.0 }
                io.clearBulkCache()
            } else runBlocking {
                robot.swerveControlLoop.target(SwerveController.Target(
                    Transform2D(
                        gm1.leftStick.let { vec2(it.xAxis.m/s,it.yAxis.m/s) },
                        gm1.rightStick.xAxis.rad/s
                    ),
                    gamepad1.left_stick_button
                ))
            }
        }
    }
}
