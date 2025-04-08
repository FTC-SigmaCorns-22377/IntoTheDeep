package sigmacorns.common.cmd

import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.schedule

class DeadlineGroup(val deadline: Command, private vararg val cmds: Command) : Command() {
    override var dependencies: Set<Command> = cmds.toSet()

    override var onCancel = {
        deadline.status = Status.CANCELLED;
        cmds.forEach { if(it.status!=Status.FINISHED) it.status = Status.CANCELLED }
    }

    /**
     * Initializes all the commands in the group in parallel using coroutines.
     */
    override suspend fun init(): Unit {
        deadline.schedule()
        cmds.forEach { it.schedule() }
    }

    /**
     * Runs all the commands in the group in parallel using coroutines.
     *
     * @return True if all commands return true, false otherwise.
     */
    override suspend fun run(): Boolean {
        if(deadline.status == Status.CANCELLED) {
            status = Status.CANCELLED
        }

        if(deadline.status == Status.CANCELLED || deadline.status == Status.CANCELLED) {
            cmds.forEach { if(it.status!=Status.FINISHED) it.status = Status.CANCELLED }
        }

        return deadline.status == Status.FINISHED
    }
}

fun deadline(deadline: Command, vararg cmds: Command) = DeadlineGroup(deadline,*cmds)