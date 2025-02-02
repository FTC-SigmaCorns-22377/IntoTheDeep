package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.derived.Radian
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.instant
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Tuning

fun liftCommand(robot: Robot, dist: Metre) = robot.liftCommandSlot.schedule(robot.slides.follow {
    DiffyOutputPose(robot.slides.t.axis1.also { println("WHEN SETTING LIFT EXTENSION = $it") }, dist)
})

fun armCommand(robot: Robot, arm: Radian, wrist: Radian) = robot.armCommandSlot.schedule(robot.arm.follow(
    DiffyOutputPose(arm,wrist))
)

fun depoCommand(robot: Robot, liftPose: LiftPose)
    = armCommand(robot,liftPose.arm,liftPose.wrist) + liftCommand(robot, liftPose.lift)

fun activeIntake(robot: Robot, active: Boolean) = instant { robot.active.updatePort(if(active) Tuning.ACTIVE_POWER else 0.0); true }