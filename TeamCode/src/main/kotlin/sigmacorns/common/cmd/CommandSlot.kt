package sigmacorns.common.cmd

import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.schedule

class CommandSlot: Command() {
    private var curCmd: Command? = null
    private var interruptible = true
    private var runningProxy: Command? = null

    override suspend fun run(): Boolean {
//        if(runningProxy?.status==Status.CANCELLED) {
//            curCmd!!.status = Status.CANCELLED
//            runningProxy = null
//        }
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
        onCancel = { if(command.status!=Status.FINISHED) command.status = Status.CANCELLED }
        init { if(!tryPut(command, interruptible)) status = Status.CANCELLED; }
        loop { if(command.status==Status.CANCELLED) status=Status.CANCELLED }
        finishWhen { command.status==Status.FINISHED }
    }

    fun lock() = cmd {
        instant { tryPut(this,false) }
    }.setInverse(cmd {
        instant { if(curCmd==this) curCmd = null }
    })
}