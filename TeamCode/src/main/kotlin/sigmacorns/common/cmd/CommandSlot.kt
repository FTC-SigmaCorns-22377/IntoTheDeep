package sigmacorns.common.cmd

import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.schedule

class CommandSlot: Command() {
    var curCmd: Command? = null
    private var interruptible = true

    override fun run(): Boolean {
        return false
    }

    private fun tryPut(cmd: Command, interruptible: Boolean = true): Boolean {
        if(curCmd != null && !interruptible) return false
        curCmd?.status = Status.CANCELLED
        curCmd = cmd
        cmd.schedule()
        this.interruptible = interruptible
        return true
    }

    fun register(command: Command, interruptible: Boolean = true) = cmd {
        dependencies += command
        onCancel = { if(command.status!=Status.FINISHED) command.status = Status.CANCELLED }
        init { if(!tryPut(command, interruptible)) status = Status.CANCELLED; }
        loop { if(command.status==Status.CANCELLED) status=Status.CANCELLED }
        finishWhen { command.status==Status.FINISHED }
    }.name("registerSlot(${command.name})")

    fun lock() = cmd {
        instant { tryPut(this,false) }
    }.setInverse(cmd {
        instant { if(curCmd==this) curCmd = null }
    })
}