package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.rotate
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z
import net.unnamedrobotics.lib.rerun.archetypes.Points3D
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.Constants
import sigmacorns.common.Robot
import sigmacorns.common.Tuning
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.arm.armControlLoop
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.SwerveHeadingController
import sigmacorns.common.subsystems.swerve.swerveHeadingControlLoop
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ClawMachineTest: SimOrHardwareOpMode() {
    override val initialScoringPose = ScoringPose(
        vec2(0.m,0.m),
        0.degrees,
        406.4.mm,
        90.degrees,
        0.rad,0.rad
    )

    override fun runOpMode(io: SigmaIO) {

        val robot = Robot(io)

        robot.addLoop(robot.armControlLoop)
        val headingControlLoop = swerveHeadingControlLoop(robot.swerveControlLoop.controller)
        robot.addLoop(headingControlLoop)
        robot.addLoop(robot.swerveLogPosControlLoop)

        gamepad1.type = Gamepad.Type.LOGITECH_F310
        gamepad2.type = Gamepad.Type.LOGITECH_F310

        waitForStart()

        robot.launchIOLoop()

        var scoringPose = initialScoringPose

        val speed = 15.cm/s
        val driveSpeed = 2.m/s
        val turnSpeed = 0.5.rad/s

        val target = ScoringKinematics.forward(scoringPose)

        val turnController = PIDController(Tuning.TELEOP_HEADING_PID)

        robot.inputLoop { dt ->
            turnController.coefficients = Tuning.TELEOP_HEADING_PID
            val p = robot.swerveControlLoop.controller.logPosition

            target.samplePos = vec3(
                target.samplePos.x - gamepad2.left_stick_y*dt*speed,
                target.samplePos.y - gamepad2.left_stick_x*dt*speed,
                target.samplePos.z + gamepad2.right_stick_x*dt*speed
            )
            target.robotPos = p.vector()

            scoringPose = ScoringKinematics.inverse(target)

            scoringPose.pivot = Constants.ARM_PIVOT_BOUNDS.apply(scoringPose.pivot)
            scoringPose.extension = Constants.ARM_EXTENSION_BOUNDS.apply(scoringPose.extension)

            runBlocking { robot.armControlLoop.target(scoringPose.armTarget(true)) }

            val xSpeed = -gamepad1.left_stick_y * driveSpeed
            val ySpeed = -gamepad1.left_stick_x * driveSpeed
            val lockWheels = gamepad1.left_stick_button

            headingControlLoop.mapTarget {
                val theta = -p.angle
                SwerveHeadingController.Target(vec2(xSpeed,ySpeed).rotate(theta.cast(rad)), scoringPose.theta)
            }

            runBlocking { robot.swerveControlLoop.withLock { robot.swerveLogPosControlLoop.withLock { robot.armControlLoop.withLock {
                rerun(robot.io.rerunConnection) {
                    log("TARGET POS") { Points3D(listOf(target.samplePos)) }
                }
            }  } } }
        }
    }
}