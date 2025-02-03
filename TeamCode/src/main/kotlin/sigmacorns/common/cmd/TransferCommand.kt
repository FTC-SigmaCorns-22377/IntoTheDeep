package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.instant
import net.unnamedrobotics.lib.command.under
import net.unnamedrobotics.lib.math2.Transform2D
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Tuning

fun transferCommand(robot: Robot): Command = instant { println("hii"); true } then under(
robot.armCommandSlot.lock() +
    robot.liftCommandSlot.lock() +
    robot.extendCommandSlot.lock() +
    robot.intakeCommandSlot.lock()
)(
    (
            //EXAMPLE:
//            cmd {
//                instant { println("whee") }
//            }
//            (robot.mecanum.follow(Transform2D(1.m/s,0.m/s,0.rad/s)) + wait())
        (robot.arm.follow(Tuning.TRANSFER_ARM) +
        robot.slides.follow(DiffyOutputPose(Tuning.TRANSFER_EXTEND,Tuning.HOVER_DIST)).timeout(4.s) +
        robot.intake.follow(Robot.IntakePositions.BACK) +
                (instant { robot.active.updatePort(0.3); true } then wait(100.ms) then instant { robot.active.updatePort(0.0); true }) +
        instant { robot.claw.updatePort(Tuning.CLAW_OPEN); true}
    ) then (robot.intake.follow(Robot.IntakePositions.OVER) + wait(200.ms))
    then wait(600.ms)
    then robot.slides.follow(DiffyOutputPose(Tuning.TRANSFER_EXTEND,Tuning.TRANSFER_LIFT)).timeout(3.s)
    then instant { robot.active.updatePort(-1.0); true }
    then wait(300.ms)
    then instant { robot.claw.updatePort(Tuning.CLAW_CLOSED); true }
    then wait(300.ms)
    then instant { robot.active.updatePort(0.0); true }
    then race(robot.slides.follow(DiffyOutputPose(Tuning.TRANSFER_EXTEND,Tuning.EXTRACT_LIFT)),
        wait(3000.ms)
    ))
).timeout(8.s)

fun Command.timeout(t: Second) = race(this, wait(t))