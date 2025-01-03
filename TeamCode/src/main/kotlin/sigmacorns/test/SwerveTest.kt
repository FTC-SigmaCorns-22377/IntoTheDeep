package sigmacorns.test

import android.icu.text.DecimalFormat
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import net.unnamedrobotics.lib.command.fsm
import net.unnamedrobotics.lib.control.circuit.controlGraph
import net.unnamedrobotics.lib.control.circuit.portTIn
import net.unnamedrobotics.lib.control.circuit.portUOut
import net.unnamedrobotics.lib.control.circuit.portXIn
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.hardware.impl.CRServoImpl
import net.unnamedrobotics.lib.hardware.impl.ServoImpl
import net.unnamedrobotics.lib.hardware.interfaces.Servo
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math.radians
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import sigmacorns.common.io.RobotIO
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.SwerveTarget
import sigmacorns.common.subsystems.swerve.tickToAngle
import kotlin.math.PI

@TeleOp
class SwerveTest: LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)
        val controller = SwerveController(PIDCoefficients(0.0,0.0,0.0))

        val timer = ElapsedTime()

        val imu = hardwareMap.get(IMU::class.java,"imu")
        val params = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        )
        imu.initialize(params)

        val stack: ArrayDeque<String> = ArrayDeque()
        var backPressed = false
        var dpadDownPressed = false
        var dpadUpPressed = false
        var dpadLeftPressed = false
        var dpadRightPressed = false
        var xPressed = false
        var yPressed = false
        var aPressed = false
        var bPressed = false
        val octoPosToModuleIs = arrayOf(1,3,2,0)
        val encodersReversed = arrayOf(1,1,1,1)
        val servosReversed = arrayOf(1,1,1,1)
        val motorsReversed = arrayOf(1,1,1,1)
        var moduleToConfigure = 0

        io.turns.forEachIndexed { i,it ->
            it.reverse(servosReversed[i]==1)
        }
        io.drives.forEachIndexed { i,it ->
            it.reverse(motorsReversed[i]==1)
        }


        val dashboard = FtcDashboard.getInstance()
        val dashTelemetry = dashboard.telemetry

        var fieldRelative = true

//        val swerveTarget = {
//            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
//            SwerveTarget(
//                Vector2(gamepad1.left_stick_x,-gamepad1.left_stick_y).rotate(if(fieldRelative) -heading.radians else 0.0.radians),
//                (gamepad1.right_trigger-gamepad1.left_trigger).toDouble(),
//                gamepad1.left_stick_button,
//            )
//        }
//
//        val modulePositions = {
//            octo.readAllPositions()
//                .run { octoPosToModuleIs.map { this[it]} }
//                .mapIndexed { i,it -> it*encodersReversed[i].toDouble() }
//                .toTypedArray()
//        }

//        val baseControlGraph = controlGraph("baseControlGraph") {
//            root(swerveTarget) connect controller.portTIn()
//            modulePositions connect controller.portXIn()
//            controller.portUOut() connect { it.turnPowers.zip(turns).map { it.second.power = it.first } }
//            controller.portUOut() connect { it.drivePowers.zip(drives).map { it.second.power = it.first } }
//        }

//        val fsm = fsm {
//            state("Base") {
//                init { turns.forEach { it.controller.pwmEnable() } }
//                run {
//                    val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
//                    val positions = octo.readAllPositions().run { octoPosToModuleIs.map {
//                        this[it]
//                    } }.mapIndexed { i,it -> it*encodersReversed[i].toDouble() }
//
//                    controller target SwerveTarget(
//                        Vector2(gamepad1.left_stick_x,-gamepad1.left_stick_y)
//                            .rotate(if(fieldRelative) -heading.radians else 0.0.radians),
//                        (gamepad1.right_trigger-gamepad1.left_trigger).toDouble(),
//                        gamepad1.left_stick_button,
//                    )
//                    controller updatePosition positions.toTypedArray()
//                    controller.update(timer.seconds()).let {
//                        it.turnPowers.zip(turns).forEach { it.second.power = it.first }
//                        it.drivePowers.zip(drives).forEach { it.second.power = it.first }
//                    }
//                }
//                transition("Reset IMU") { gamepad1.a }
//            }
//            state("Reset IMU") {
//                run { imu.resetYaw() }
//                transition("Base") { true }
//            }
//        }

        waitForStart()

        while (opModeIsActive()) {
            if(gamepad1.back && !backPressed && !stack.isEmpty()) stack.removeLast()
            backPressed = gamepad1.back

//            val positions = octo.readAllPositions().run { octoPosToModuleIs.map {
//                this[it]
//            } }.mapIndexed { i,it -> it*encodersReversed[i].toDouble() }

            val positions = io.turnEncoders.map { (it.voltage/3.3)* PI*2 }

            val telemetry = dashTelemetry

            //base case: drive regularly
            if(stack.isEmpty()) {
                //whether pwm is enabled is cached so minimum lynxcommands sent.
                val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

                controller.target = SwerveTarget(
                    Vector2(gamepad1.left_stick_x,-gamepad1.left_stick_y)
                        .rotate(if(fieldRelative) -heading.radians else 0.0.radians),
                    (gamepad1.right_trigger-gamepad1.left_trigger).toDouble(),
                    gamepad1.left_stick_button,
                )
                controller.position = positions
                controller.update(timer.seconds()).let {
                    it.drivePowers.zip(io.drives).map { it.second.power = it.first }
                    it.turnPowers.zip(io.turns).map { it.second.power = it.first }
                }

                telemetry.addData("(A) ", "reset IMU")
                telemetry.addData("(B) ", "test powers.")
                telemetry.addData("(X) ", "tune PID.")
                telemetry.addData("(Y) ","configure hardware.")
                telemetry.addData("(⯆) ","power off servos for encoder reset.")
                telemetry.addData("(⯅)","make " + (if(fieldRelative) "robot" else "field") + " relative.")

                dpadDownPressed = gamepad1.dpad_down
                dpadLeftPressed = gamepad1.dpad_left
                dpadUpPressed = gamepad1.dpad_up
                dpadRightPressed = gamepad1.dpad_right
                yPressed = gamepad1.y
                aPressed = gamepad1.a
                xPressed = gamepad1.x
                bPressed = gamepad1.b

                if(gamepad1.a) imu.resetYaw()
                else if(gamepad1.b) stack += "Test powers"
                else if(gamepad1.x) stack += "Tune PID"
                else if(gamepad1.y) stack += "Configure hardware"
                else if(gamepad1.dpad_down) stack += "Servos off"
                else if(gamepad1.dpad_up && !dpadUpPressed) fieldRelative = !fieldRelative
            } else when(stack.last()) {
                "Servos off" -> {
                    telemetry.addData("Servos are off. ","press back to return")
//                    if(gamepad1.dpad_down) octo.resetMultiplePositions(*octoPosToModuleIs.toIntArray())
                }
                "Test powers" -> {
                    telemetry.addData("Testing powers. ","press back to return")
                    telemetry.addData("(A) ","drive 1")
                    telemetry.addData("(B) ","drive 2")
                    telemetry.addData("(X) ","drive 3")
                    telemetry.addData("(Y) ","drive 4")
                    telemetry.addData("(⯅) ","turn 1")
                    telemetry.addData("(⯆) ","turn 2")
                    telemetry.addData("(⯇) ","turn 3")
                    telemetry.addData("(⯈) ","turn 4")
                    telemetry.addData("right bumper ","power multiplier")
                    telemetry.addData("(Left joystick (x,y)) ", "(turn,drive)")

                    val v = Vector2(gamepad1.left_stick_x,-gamepad1.left_stick_y)
                    telemetry.addData("turn = ", v.x)
                    telemetry.addData("drive = ", v.y)
                    if(v.magnitude>0.01) {
                        io.drives.forEach { it.power = v.y }
                        io.turns.forEach { it.power = v.x }
                    } else {
                        io.drives.zip(arrayOf(gamepad1.a,gamepad1.b,gamepad1.x,gamepad1.y)).forEach {
                            it.first.power = if(it.second) 1.0-gamepad1.right_trigger else 0.0
                        }
                        io.turns.zip(arrayOf(gamepad1.dpad_up,gamepad1.dpad_down,gamepad1.dpad_left,gamepad1.dpad_right)).forEach {
                            it.first.power = if(it.second) 1.0-gamepad1.right_trigger else 0.0
                        }
                    }
                }
                "Configure hardware" -> {
                    telemetry.addData("Configuring hardware. ","press back to return")
                    telemetry.addData("drive directions: ", io.drives.map { it.getReversed() }.joinToString(prefix= "[", postfix = "]"))
//                    telemetry.addData("turn directions: ", io.turns.map { it. }.joinToString(prefix= "[", postfix = "]"))
                    telemetry.addData("module indices", octoPosToModuleIs.joinToString(prefix= "[", postfix = "]"))
                    telemetry.addData("configuring module ", moduleToConfigure+1)
                    telemetry.addData("(⯅) ","module++")
                    telemetry.addData("(⯆) ","module--")
                    telemetry.addData("(⯇) ","decrease octoquad port")
                    telemetry.addData("(⯈) ","increase octoquad port")
                    telemetry.addData("(A) ","invert drive direction")
                    telemetry.addData("(B) ","invert turn direction")
                    telemetry.addData("(Y) ","invert turn encoder direction")
                    telemetry.addData("(Left joystick (x,y)) ", "(turn,drive)")

                    if(gamepad1.dpad_up && !dpadUpPressed) moduleToConfigure = (moduleToConfigure+1)%4
                    if(gamepad1.dpad_down && !dpadDownPressed) moduleToConfigure = (moduleToConfigure-1).mod(4)
                    if(gamepad1.dpad_right && !dpadRightPressed) octoPosToModuleIs[moduleToConfigure] = (octoPosToModuleIs[moduleToConfigure]+1).mod(8)
                    if(gamepad1.dpad_left && !dpadLeftPressed) octoPosToModuleIs[moduleToConfigure] = (octoPosToModuleIs[moduleToConfigure]-1).mod(8)
                    if(gamepad1.a && !aPressed) io.drives[moduleToConfigure].reverse(!io.drives[moduleToConfigure].getReversed())
                    if(gamepad1.b && !bPressed) io.turns[moduleToConfigure].reverse((io.turns[moduleToConfigure] as CRServoImpl).device.direction.inverted() == DcMotorSimple.Direction.FORWARD)
                    if(gamepad1.y && !yPressed) encodersReversed[moduleToConfigure] = -1*encodersReversed[moduleToConfigure]
                    val v = Vector2(gamepad1.left_stick_x,-gamepad1.left_stick_y)
                    telemetry.addData("turn = ", v.x)
                    telemetry.addData("drive = ", v.y)
                    if(v.magnitude>0.01) {
                        io.drives.forEach { it.power = v.y }
                        io.turns.forEach { it.power = v.x }
                    }

                    dpadDownPressed = gamepad1.dpad_down
                    dpadLeftPressed = gamepad1.dpad_left
                    dpadUpPressed = gamepad1.dpad_up
                    dpadRightPressed = gamepad1.dpad_right
                    aPressed = gamepad1.a
                    bPressed = gamepad1.b
                    yPressed = gamepad1.y
                }
            }

            controller.modules.mapIndexed { i,module ->
                val fmt = DecimalFormat("#,###.##")
                val caption = "Wheel " + (i+1) + ": "
                telemetry.addData(caption + "target angle = ", fmt.format(module.target.v.angleFromOrigin))
                telemetry.addData(caption + "pos = ", fmt.format(positions[i]))
                telemetry.addData(caption + "power = ",  fmt.format(io.turns[i].power))
                telemetry.addData(caption + "angle = ", fmt.format(tickToAngle(positions[i].toInt())))
                telemetry.addData("vs [$i]",controller.moduleTargetVectors[i])
            }
            telemetry.update()
        }
    }
}
