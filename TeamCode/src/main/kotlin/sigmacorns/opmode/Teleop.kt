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
        0.degrees,
        406.4.mm,
        90.degrees,
        0.rad,0.rad
    )

    private val slewRateLimiter = FlippingSlewRateLimiter(Tuning.SWERVE_MAX_SLEW_RATE)
    private val headingController = PIDController(Tuning.TELEOP_HEADING_PID)

    var targetHeading: Expression = initialScoringPose.theta

    override fun runOpMode(io: SigmaIO) {
        io.rerunConnection.disabled = true

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

        val normalAngularMaxSpeed = 0.5 * robot.topAngularSpeed()
        val slowAngularMaxSpeed = (0.5) * normalAngularMaxSpeed

        val armPivotNormalSpeed = 1.0.rad/s
        val armPivotSlowSpeed = armPivotNormalSpeed * 0.5

        val armExtensionNormalSpeed = 1.0.m/s
        val armExtensionSlowSpeed = armExtensionNormalSpeed * 0.5

        val armSpeed = 6.cm/s

        val clawRollSpeed = 2.rad/s

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


        robot.inputLoop { dt ->
            gm1.periodic()
            gm2.periodic()
            runBlocking {
                val pos = swerve.controller.logPosition
                val vel = swerve.controller.logVelocity
                val armPos = ScoringKinematics.forward(ScoringPose(armTarget))

                slewRateLimiter.maxRate = Tuning.SWERVE_MAX_SLEW_RATE
                headingController.coefficients = Tuning.TELEOP_HEADING_PID

                //swerve speed controls
                isSlowMode = gm1.x.isToggled
                val maxSpeed = if(isSlowMode) slowMaxSpeed else normalMaxSpeed
                val angularMaxSpeed = if(isSlowMode) slowAngularMaxSpeed else normalAngularMaxSpeed

                //swerve controls
                val xSpeed = -gm1.leftStick.yAxis * maxSpeed
                val ySpeed = -gm1.leftStick.xAxis * maxSpeed
                val angularSpeed = -gamepad1.right_stick_x.toDouble() * angularMaxSpeed
                val lockWheels = gamepad1.left_stick_button

//                targetHeading += angularSpeed*dt
//
//                val diff = (targetHeading-pos.angle).normalizeRadian()
//                    .map { it.clampMagnitude(Tuning.TELEOP_TARGET_HEADING_MAX_DIFF) }
//                targetHeading = pos.angle + diff
//
//                val angularPower = headingController.updateStateless(
//                    dt.value,
//                    pos.angle.value,
//                    targetHeading.value
//                ).rad/s

                var speed = vec2(xSpeed,ySpeed)
//                if(speed.magnitude().value > Tuning.MIN_VEL_SLEW_LIMIT) {
//                    val angle = slewRateLimiter.updateStateless(
//                        dt.value,
//                        vel.vector().theta().value,
//                        speed.theta().value
//                    ).rad
//                    speed = polar(speed.magnitude(),angle)
//                }

                // field relative wooo
                swerve.mapTarget {
                    val theta = -it.logPosition.angle
                    SwerveController.Target(Transform2D(speed.rotate(theta.cast(rad)), angularSpeed), lockWheels)
                }

                if(gm1.start.isJustPressed)
                    if(!SIM) (io as RobotIO).pinpoint.reset()

//                //speed control
//                if(gm2.x.isJustPressed) isArmSlowMode = !isArmSlowMode
//                val armPivotSpeed = if(isArmSlowMode) armPivotSlowSpeed else armPivotNormalSpeed
//                val armExtensionSpeed = if(isArmSlowMode) armExtensionSlowSpeed else armExtensionNormalSpeed
//
//                //manual arm controls
//                armTarget.pivot = (armTarget.pivot + (gm2.leftStick.yAxis * dt * armPivotSpeed)).cast(rad)
//                armTarget.pivot = Constants.ARM_PIVOT_BOUNDS.apply(armTarget.pivot)
//
//                armTarget.extension = (armTarget.extension + (gm2.rightStick.yAxis * dt * armExtensionSpeed)).cast(m)
//                armTarget.extension = Constants.ARM_EXTENSION_BOUNDS.apply(armTarget.extension)

                armTarget = ScoringKinematics.inverse(armPos.also {
                    it.samplePos = vec3(
                        it.samplePos.x + gm2.leftStick.xAxis*armSpeed*dt,
                        it.samplePos.y,
                        it.samplePos.z + gm2.leftStick.yAxis*armSpeed*dt
                    )
                }).armTarget(armTarget.isOpen)

                armTarget.extension = Constants.ARM_EXTENSION_BOUNDS.apply(armTarget.extension)
                armTarget.pivot = Constants.ARM_PIVOT_BOUNDS.apply(armTarget.pivot)

                //roll
                if(gm1.leftBumper.isPressed || gm2.leftBumper.isPressed)
                    armTarget.roll = Constants.CLAW_ROLL_BOUNDS.apply((armTarget.roll - clawRollSpeed*dt).cast(rad))
                if(gm1.rightBumper.isPressed || gm2.rightBumper.isPressed)
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