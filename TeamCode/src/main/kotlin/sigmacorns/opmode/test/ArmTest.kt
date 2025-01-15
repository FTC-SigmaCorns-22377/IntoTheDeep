package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Constants
import sigmacorns.common.Robot
import sigmacorns.common.Tuning
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.arm.boxTubeKinematics
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp
class ArmTest: SimOrHardwareOpMode() {
    override val initialScoringPose = ScoringPose(
        vec2(0.m,0.m),
        0.degrees,
        400.mm,
        90.degrees,
        0.rad,0.rad
    )

    override fun runOpMode(io: SigmaIO) {
        val armManualPivotSpeed = 30.degrees/s
        val armManualExtensionSpeed = 0.1.m/s

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

        robot.addLoop(robot.armControlLoop)

        val gm1 = GamepadEx(gamepad1)

        waitForStart()

        robot.launchIOLoop()


        gm1.a.onToggle(cmd {
            init { robot.armControlLoop.target(ArmTarget(
                0.rad,600.mm,0.rad,0.rad,false
            )) }
        })

        gm1.a.onUntoggle(cmd {
            init { robot.armControlLoop.target(ArmTarget(
                65.rad,400.mm,0.rad,0.rad,false
            )) }
        })

        robot.inputLoop { dt ->
            gm1.periodic()
            Scheduler.tick()

            robot.armControlLoop.mapTarget {
                it.armDiffyController = it.pidDiffyController(boxTubeKinematics,Tuning.ARM_PIVOT_PID,Tuning.ARM_EXTENSION_PID)
                it.target.let { target ->
                    ArmTarget(
                        Constants.ARM_PIVOT_BOUNDS.apply((target.pivot + gm1.leftStick.yAxis*armManualPivotSpeed*dt).cast(rad)),
                        Constants.ARM_EXTENSION_BOUNDS.apply((target.extension + gm1.leftStick.xAxis*armManualExtensionSpeed*dt).cast(m)),
                        target.pitch,
                        target.roll,false
//                        gm1.a.isToggled
                    )
                }
            }
        }
    }
}