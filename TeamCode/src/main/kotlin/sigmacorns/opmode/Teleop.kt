package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.polar
import net.unnamedrobotics.lib.math2.rotate
import net.unnamedrobotics.lib.math2.unitless
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.vector
import net.unnamedrobotics.lib.math2.z
import sigmacorns.common.Constants
import sigmacorns.common.Robot
import sigmacorns.common.ScoringPresets
import sigmacorns.common.Tuning
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.swerve.FlippingSlewRateLimiter
import sigmacorns.common.subsystems.swerve.SwerveController

@TeleOp
class Teleop: SimOrHardwareOpMode() {
    override val initialScoringPose = ScoringPose(
        vec2(0.m,0.m),
        90.degrees,
        406.4.mm,
        90.degrees,
        0.rad,0.rad
    )

    private val slewRateLimiter = FlippingSlewRateLimiter(Tuning.SWERVE_MAX_SLEW_RATE)
    private val headingController = PIDController(Tuning.TELEOP_HEADING_PID)

    var targetHeading: Expression = initialScoringPose.theta

    override fun runOpMode(io: SigmaIO) {
//        io.rerunConnection.disabled = true

        val robot = Robot(io)

        runBlocking {
            robot.armControlLoop.target(ArmTarget(
                initialScoringPose.pivot,
                initialScoringPose.extension,
                0.rad,
                0.rad,
                false
            ))
        }

        val normalMaxSpeed = 1.0 * robot.topSpeed()
        val slowMaxSpeed = (0.5) * normalMaxSpeed

        val normalAngularMaxSpeed = 0.4 * robot.topAngularSpeed()
        val slowAngularMaxSpeed = (0.5) * normalAngularMaxSpeed

        val armPivotNormalSpeed = 0.5.rad/s
        val armPivotSlowSpeed = armPivotNormalSpeed * 0.5

        val armExtensionNormalSpeed = 25.cm/s
        val armExtensionSlowSpeed = armExtensionNormalSpeed * 0.5

        val clawRollSpeed = 6.rad/s

        gamepad1.type = Gamepad.Type.LOGITECH_F310
        gamepad2.type = Gamepad.Type.LOGITECH_F310

        val gm1 = GamepadEx(gamepad1)
        val gm2 = GamepadEx(gamepad2)

        val armLoop = robot.armControlLoop
        val swerve = robot.swerveControlLoop

        io.addLoop(swerve)
        io.addLoop(armLoop)
        io.addLoop(robot.swerveLogPosControlLoop)
        io.addLoop(robot.swerveLogVelControlLoop)

        var isSlowMode = false

        var isArmSlowMode = false
        var armTarget = initialScoringPose.armTarget(true)

        val baseDistance = 400.mm

        waitForStart()
        robot.launchIOLoop()

        var wasA1Pressed = false
        var wasA2Pressed = false

        var wasXPressed = false

        robot.inputLoop { dt ->
            gm1.periodic()
            gm2.periodic()
            runBlocking {
                slewRateLimiter.maxRate = Tuning.SWERVE_MAX_SLEW_RATE
                headingController.coefficients = Tuning.TELEOP_HEADING_PID

                //swerve speed controls
                if(gamepad1.x && !wasXPressed) isSlowMode = !isSlowMode
                wasXPressed = gamepad1.x

                val maxSpeed = if(isSlowMode) slowMaxSpeed else normalMaxSpeed
                val angularMaxSpeed = if(isSlowMode) slowAngularMaxSpeed else normalAngularMaxSpeed

                //swerve controls
                val xSpeed = -gm1.leftStick.yAxis * maxSpeed
                val ySpeed = -gm1.leftStick.xAxis * maxSpeed
                val angularSpeed = -gamepad1.right_stick_x.toDouble() * angularMaxSpeed
                val lockWheels = gamepad1.left_stick_button

                val speed = vec2(xSpeed,ySpeed)

                // field relative wooo
                swerve.mapTarget {
                    val theta = -it.logPosition.angle
                    SwerveController.Target(Transform2D(speed.rotate(theta.cast(rad)), angularSpeed), lockWheels)
                }

                if(gamepad1.start)
                    if(!SIM) (io as RobotIO).pinpoint.reset()

                armTarget.extension = Constants.ARM_EXTENSION_BOUNDS.apply((armTarget.extension - gamepad2.left_stick_y*armExtensionNormalSpeed*dt).cast(m))
                armTarget.pivot = Constants.ARM_PIVOT_BOUNDS.apply((armTarget.pivot + gamepad2.right_stick_y*armPivotNormalSpeed*dt).cast(rad))

                //roll
                if(gamepad1.left_bumper || gamepad2.left_bumper)
                    armTarget.roll = Constants.CLAW_ROLL_BOUNDS.apply((armTarget.roll - clawRollSpeed*dt).cast(rad))
                if(gamepad1.right_bumper || gamepad2.right_bumper)
                    armTarget.roll = Constants.CLAW_ROLL_BOUNDS.apply((armTarget.roll + clawRollSpeed*dt).cast(rad))

                //claw
                if((gamepad1.a && !wasA1Pressed) || (gamepad2.a && !wasA2Pressed))
                    armTarget.isOpen = !armTarget.isOpen

                wasA1Pressed = gamepad1.a
                wasA2Pressed = gamepad2.a

                if(gamepad1.b || gamepad2.b)
                    armTarget = ScoringPresets.placeLowSpecimen().armTarget(armTarget.isOpen).also { println("SET LOW")}

                if(gamepad1.y || gamepad2.y)
                    armTarget = ScoringPresets.placeHighSpecimen().armTarget(armTarget.isOpen).also { println("SET HIGH")}
                if(gamepad2.x)
                    armTarget = ScoringPresets.placeOverSubmersible(baseDistance).armTarget(armTarget.isOpen)

                if(gamepad1.dpad_down || gamepad2.dpad_down)
                    armTarget = ScoringPresets.placeLowSample().armTarget(armTarget.isOpen)
                if(gamepad1.dpad_up || gamepad2.dpad_up)
                    armTarget = ScoringPresets.placeHighSample().armTarget(armTarget.isOpen)
//                println("ARM TARGET: ${armTarget.pivot}, ${armTarget.extension}")

                armLoop.target(armTarget)
            }
        }
    }

}