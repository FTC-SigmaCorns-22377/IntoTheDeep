package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.Radian
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Tuning

fun liftCommand(robot: Robot, dist: Metre, lock: Boolean = true) = robot.slides.follow {
    DiffyOutputPose(robot.slides.t.axis1, dist)
}.let {
    if(lock) robot.liftCommandSlot.register(it) else it
}

fun armCommand(robot: Robot, arm: Radian, wrist: Radian, lock: Boolean = true) = robot.arm.follow(
    DiffyOutputPose(arm,wrist)
).let {
    if(lock) robot.liftCommandSlot.register(it) else series(instant { println("STARTED ARM") }, it, instant { println("OH SHIT BRO") })
}

fun clawCommand(robot: Robot, closed: Boolean)
    = instant { robot.claw.updatePort(if(closed) Tuning.CLAW_CLOSED else Tuning.CLAW_OPEN) } + wait(Tuning.CLAW_TIME)

fun depoCommand(robot: Robot, liftPose: LiftPose, lock: Boolean = true)
    = (armCommand(robot,liftPose.arm,liftPose.wrist,lock).name("arm") + liftCommand(robot, liftPose.lift,lock).name("lift")).name("depo")

fun depoCommand(robot: Robot, positions: ScorePosition) = depoCommand(robot,positions.x)

fun instant(f: ()->Unit) = cmd {instant(f)}

