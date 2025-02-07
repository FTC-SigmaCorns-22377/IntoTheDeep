package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.cm
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.coroutineScope
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.initCommand
import net.unnamedrobotics.lib.command.runCommand

class DeadlineGroup(val deadline: Command, private vararg val cmds: Command) : Command() {
    /**
     * Initializes all the commands in the group in parallel using coroutines.
     */
    override suspend fun init(): Unit = coroutineScope {
        initCommand(deadline)
        cmds.map { async { initCommand(it) } }.awaitAll()
    }

    /**
     * Runs all the commands in the group in parallel using coroutines.
     *
     * @return True if all commands return true, false otherwise.
     */
    override suspend fun run(): Boolean = coroutineScope {
        cmds.map { async { runCommand(it) } }.awaitAll()
        (deadline.status == Status.FINISHED).also {
            if(it) cmds.forEach { if(it.status!=Status.FINISHED) it.status = Status.CANCELLED }
        }
    }
}

fun deadline(deadline: Command, vararg cmds: Command) = DeadlineGroup(deadline,*cmds)