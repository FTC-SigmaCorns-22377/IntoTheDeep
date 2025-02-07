package sigmacorns.common.cmd

import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.schedule

class CommandSlot: Command() {
    private var curCmd: Command? = null
    private var interruptible = true

    override suspend fun run(): Boolean {
        return false
    }

    fun tryPut(cmd: Command, interruptible: Boolean = true): Boolean {
        if(curCmd != null && !interruptible) return false
        curCmd?.status = Status.CANCELLED
        curCmd = cmd
        cmd.schedule()
        this.interruptible = interruptible
        return true
    }

    fun register(command: Command, interruptible: Boolean = true) = cmd {
        init { tryPut(command, interruptible) }
        finishWhen { command.status==Status.FINISHED }
    }

    fun lock() = cmd {
        instant { tryPut(this,false) }
    }.setInverse(cmd {
        instant { if(curCmd==this) curCmd = null }
    })
}