package sigmacorns.common.cmd

import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.initCommand
import net.unnamedrobotics.lib.command.runCommand
import net.unnamedrobotics.lib.command.schedule

class RaceGroup(private vararg val cmds: Command) : Command() {
    override var dependencies: Set<Command> = cmds.toSet()
    override var onCancel = { cmds.forEach { if(it.status!=Status.FINISHED) it.status = Status.CANCELLED } }

    /**
     * Initializes all the commands in the group in parallel using coroutines.
     */
    override suspend fun init(): Unit {
        cmds.forEach { it.schedule() }
    }

    /**
     * Runs all the commands in the group in parallel using coroutines.
     *
     * @return True if all commands return true, false otherwise.
     */
    override suspend fun run(): Boolean {
        return cmds.any { it.status == Status.FINISHED }.also {
            if(it) cmds.forEach { if(it.status!=Status.FINISHED) it.status = Status.CANCELLED }
        }
    }
}

/**
 * Creates a ParallelGroup with the given commands.
 *
 * @param cmds The commands to be run in parallel.
 * @return A ParallelGroup containing the given commands.
 */
fun race(vararg cmds: Command) = RaceGroup(*cmds)
