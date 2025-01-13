package sigmacorns.opmode

import com.outoftheboxrobotics.photoncore.PhotonCore.state
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.specialunits.t
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.instant
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.Constants
import sigmacorns.common.Robot
import sigmacorns.common.Tuning
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ArmController
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ClawTarget
import sigmacorns.common.subsystems.arm.armControlLoop
import sigmacorns.common.subsystems.swerve.ModuleController
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import kotlin.math.PI

@TeleOp
class Teleop: LinearOpMode() {

    val normalMaxSpeed = 0.m/s
    val slowMaxSpeed = (0.5) * normalMaxSpeed

    val normalAngularMaxSpeed = 0.rad/s
    val slowAngularMaxSpeed = (0.5) * normalAngularMaxSpeed

    override fun runOpMode() {

        val io = RobotIO(hardwareMap)
        val robot = Robot(io)

        val swerveController = SwerveController(
            ModuleController(
                Tuning.SWERVE_MODULE_PID
            ),robot.drivebase)

        val gm1 = GamepadEx(gamepad1)
        val swerveLoop = swerveControlLoop(swerveController)
        io.addLoop(swerveLoop)

        val gm2 = GamepadEx(gamepad2)
        val armLoop = armControlLoop()
        //io.addLoop(armLoop)

        var lastTime = Clock.seconds

        var isSlowMode = false

        var isArmSlowMode = false
        var armSpeedCoefficient = 1.0
        var pivot = 0.rad
        var extension = 0.m
        var pitch = 0.rad
        var roll = 0.rad
        var isClawOpen = false

        waitForStart()
        robot.launchIOLoop()

        while (opModeIsActive()) {
            runBlocking {
                // time step for additive controls
                val t = Clock.seconds
                val dt = (t - lastTime)/1000
                lastTime = t

                //swerve controls
                if(gm1.x.isJustPressed) isSlowMode = !isSlowMode
                val maxSpeed = if(isSlowMode) slowMaxSpeed else normalMaxSpeed
                val angularMaxSpeed = if(isSlowMode) slowAngularMaxSpeed else normalAngularMaxSpeed
                val xSpeed = gm1.leftStick.xAxis * maxSpeed
                val ySpeed = gm1.leftStick.yAxis * maxSpeed
                val angularSpeed = gm1.rightStick.xAxis * angularMaxSpeed
                val swerveLoopTarget = SwerveController.Target(Transform2D(xSpeed, ySpeed, angularSpeed), gamepad1.left_stick_button)
                swerveLoop.target(swerveLoopTarget)

                //arm controls
                if(gm2.x.isJustPressed) isArmSlowMode = !isArmSlowMode
                armSpeedCoefficient = if(isArmSlowMode) 1.0 else 0.5

                pivot = (pivot + (gm2.leftStick.yAxis * dt * armSpeedCoefficient * PI/2).rad).cast(rad)
                pivot = Constants.ARM_PIVOT_BOUNDS.apply(pivot)

                extension = if (gm2.a.isPressed) (extension + (dt * armSpeedCoefficient * 0.8).m).cast(m) else if (gm2.b.isPressed) (extension - (dt * 0.8).m).cast(m) else extension
                extension = Constants.ARM_EXTENSION_BOUNDS.apply(extension)

                pitch = (pitch + (-gm2.rightStick.yAxis * armSpeedCoefficient * dt * PI/2).rad).cast(rad)

                roll = if (gm2.leftBumper.isPressed) (roll + (dt * armSpeedCoefficient * 0.1).rad).cast(rad) else if (gm2.rightBumper.isPressed) (roll + (dt * 0.1).rad).cast(rad) else roll

                isClawOpen = gm2.rightTrigger.isPressed

                val armTarget = ArmTarget(pivot, extension, pitch, roll, isClawOpen)
                armLoop.target(armTarget)
            }

            gm1.periodic()
            gm2.periodic()
        }
    }
}