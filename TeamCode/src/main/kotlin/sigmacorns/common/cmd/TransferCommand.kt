package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.under
import sigmacorns.common.Robot
import sigmacorns.constants.Tuning

fun transferCommand(robot: Robot): Command = under(
robot.armCommandSlot.lock() +
    robot.liftCommandSlot.lock() +
    robot.extendCommandSlot.lock() +
    robot.intakeCommandSlot.lock()
)(
    series(
        //before, get everything in the position to transfer
        (
            depoCommand(robot,Tuning.TRANSFER_HOVER_POSE,false) +
            robot.intake.follow(Tuning.IntakePosition.BACK) +
            clawCommand(robot,false) +
            // stop rollers quickly, may want to replace this with running under encoders.
            (
                instant { robot.active.updatePort(Tuning.ACTIVE_STOP_POWER) } then
                wait(Tuning.ACTIVE_STOP_TIME) then
                instant { robot.active.updatePort(0.0) }
            )
        ).name("pre-transfer").timeout(2.s),

        // we've retracted past the bar, can flip intake to transfer pos now
        retract(robot),
        robot.intake.follow(Tuning.IntakePosition.OVER),
        // push the sample out of the top into the open claw
        (depoCommand(robot,Tuning.TRANSFER_POSE) +
                (
                    instant { robot.active.updatePort(Tuning.ACTIVE_POWER) } then
                    wait(Tuning.TRANSFER_ACTIVE_TIME) then
                    instant { robot.active.updatePort(0.0) }
                )).name("transfer-push").timeout(2.s),

        // grab the sample
        clawCommand(robot,true),

        // pull out the sample
        depoCommand(robot,Tuning.TRANSFER_EXTRACT_POSE).name("transfer-extract").timeout(3.s),
        depoCommand(robot, Tuning.postTransferPose).name("awesome-pose").timeout(2.s)
    )
).name("transfer").timeout(5.s)

fun Command.timeout(t: Second) = race(this, wait(t) then instant { println("WARNING: COMMAND  ${this.name}(${this.hashCode()}) TIMED OUT") })
fun Command.name(name: String) = this.also { it.name = name }