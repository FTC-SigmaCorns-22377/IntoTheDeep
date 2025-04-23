package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.Radian
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.wait
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Tuning

fun liftCommand(robot: Robot, dist: Metre, lock: Boolean = true) = robot.slides.follow {
    DiffyOutputPose(robot.slides.t.axis1, dist)
}.name("lift").let {
    if(lock) robot.liftCommandSlot.register(it) else it
}

fun armCommand(robot: Robot, arm: Radian, wrist: Radian, lock: Boolean = true) = robot.arm.follow(
    DiffyOutputPose(arm,wrist)
).name("arm").let {
    if(lock) robot.liftCommandSlot.register(it) else it
}

fun resetDepo(robot: Robot) = cmd {
    init { robot.slidesController.bypassAxis2 = Tuning.SLIDE_RESET_POWER }
    finishWhen { robot.io.intakeLimitTriggered() }
}.timeout(3.s) then instant {
    robot.slidesController.bypassAxis2=null
}

fun clawCommand(robot: Robot, closed: Boolean)
    = wait {
        val targetClosedBefore = robot.claw == Tuning.CLAW_CLOSED
        if(targetClosedBefore!=closed) Tuning.CLAW_TIME else 0.s
    } + instant {
        robot.claw = if(closed) Tuning.CLAW_CLOSED else Tuning.CLAW_OPEN
    }

fun depoCommand(robot: Robot, liftPose: LiftPose, lock: Boolean = true)
    = parallel(
        armCommand(robot,liftPose.arm,liftPose.wrist,lock).name("arm"),
        liftCommand(robot, liftPose.lift,lock).name("lift")
    ).name("depo")

fun depoCommand(robot: Robot, positions: ScorePosition) = depoCommand(robot,positions.x)

fun instant(f: ()->Unit) = cmd { instant(f) }

