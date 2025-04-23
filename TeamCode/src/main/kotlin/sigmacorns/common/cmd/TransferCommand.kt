package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.race
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.under
import sigmacorns.common.Robot
import sigmacorns.constants.FlapPosition
import sigmacorns.constants.Tuning

var runningTransferCommand: Command? = null
fun transferCommand(robot: Robot): Command = under(
robot.armCommandSlot.lock() +
    robot.liftCommandSlot.lock() +
    robot.extendCommandSlot.lock() +
    robot.intakeCommandSlot.lock()
)(
    series(
        instant { runningAutoInputCmd?.status = Status.CANCELLED; runningAutoInputCmd = null },
        //before, get everything in the position to transfer
        parallel(
            depoCommand(robot,Tuning.TRANSFER_HOVER_POSE,false),
            clawCommand(robot,false),
            flapCommand(robot, FlapPosition.OPEN),
            // we've retracted past the bar, can flip intake to transfer pos now
            retract(robot,false),
        ).name("pre-transfer").timeout(2.3.s),

        // push the sample out of the top into the open claw
        depoCommand(robot,Tuning.TRANSFER_POSE,false).name("transfer-push"),
        // grab the sample
        clawCommand(robot,true),

        // pull out the sample
        depoCommand(robot,Tuning.TRANSFER_EXTRACT_POSE,false).name("transfer-extract"),
    )
).name("transfer").let {
    series(
        instant {
            if(runningTransferCommand!=it) runningTransferCommand?.status = Status.CANCELLED
            runningTransferCommand = it
        },
        it
    )
}

fun Command.timeout(t: Second) = race(this, wait(t) then instant { println("WARNING: COMMAND  ${this.name}(${this.hashCode()}) TIMED OUT") })
fun Command.name(name: String) = this.also { it.name = name }