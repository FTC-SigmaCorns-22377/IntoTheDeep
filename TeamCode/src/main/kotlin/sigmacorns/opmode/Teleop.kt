package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.Constants
import sigmacorns.common.Robot
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveLogPosControlLoop
import kotlin.math.PI

@TeleOp
class Teleop: SimOrHardwareOpMode() {
    override val initialScoringPose: ScoringPose = ScoringPose(
        vec2(0.m,0.m),0.rad,400.mm,90.degrees,0.rad,0.rad,
    )

    override fun runOpMode(io: SigmaIO) {

        //TODO: test why rerun dosent fully disable (some rogue call)`
//        io.rerunConnection.disabled = true
        val robot = Robot(io)

        val normalMaxSpeed = 1.0 * robot.topSpeed()
        val slowMaxSpeed = (0.5) * normalMaxSpeed

        val normalAngularMaxSpeed = 0.7 * robot.topAngularSpeed()
        val slowAngularMaxSpeed = (0.5) * normalAngularMaxSpeed

        gamepad1.type = Gamepad.Type.LOGITECH_F310

        val gm1 = GamepadEx(gamepad1)

        val swerve = robot.swerveControlLoop
//        swerve.write.set(false)
        if(SIM) io.addLoop(robot.swerveLogPosControlLoop)
        if(SIM) io.addLoop(robot.swerveLogVelControlLoop)
        io.addLoop(swerve)

        val gm2 = GamepadEx(gamepad2)
        val armLoop = robot.armControlLoop
        //io.addLoop(armLoop)

        var lastTime = io.time().value

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

        robot.inputLoop {
            runBlocking {
//                io.turnEncoders.forEachIndexed { i,it -> println("TURN ENCODER $i = ${it.voltage}") }
                // time step for additive controls
                val t = Clock.seconds
                val dt = (t - lastTime)/1000
                lastTime = t

                //swerve controls
                if(gm1.x.isJustPressed) isSlowMode = !isSlowMode
                val maxSpeed = if(isSlowMode) slowMaxSpeed else normalMaxSpeed
                val angularMaxSpeed = if(isSlowMode) slowAngularMaxSpeed else normalAngularMaxSpeed
                val xSpeed = -gm1.leftStick.yAxis * maxSpeed
                val ySpeed = -gm1.leftStick.xAxis * maxSpeed
                val angularSpeed = gamepad1.right_stick_y.toDouble() * angularMaxSpeed
                val swerveLoopTarget = SwerveController.Target(Transform2D(xSpeed, ySpeed, angularSpeed), gamepad1.left_stick_button)
                swerve.target(swerveLoopTarget)

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
            Thread.yield()
        }
    }

}