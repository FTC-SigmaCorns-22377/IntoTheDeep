package sigmacorns

import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.s
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.util.Clock
import org.junit.Test
import sigmacorns.common.io.SimIO
import sigmacorns.common.simutil.GamepadInput
import sigmacorns.common.simutil.KeyboardGamepads
import sigmacorns.opmode.REALTIME
import sigmacorns.opmode.SIM
import sigmacorns.opmode.SimOrHardwareOpMode
import sigmacorns.opmode.Teleop
import sigmacorns.opmode.TransferTest

class OpModeTest {
    val useKeyboardInput = true
    val useGamepadInput = false
    val maxTime = 200.s


    @Test
    fun test() {
        SIM = true
        REALTIME = true
        val opMode = Teleop()

        opMode.gamepad1 = Gamepad()
        opMode.gamepad2 = Gamepad()
        var keyListener: KeyboardGamepads? = null
        var gamepadInput: GamepadInput? = null

        if(useKeyboardInput) {
            keyListener = KeyboardGamepads(opMode.gamepad1, opMode.gamepad2)
            keyListener.startListener()
        }

        if(useGamepadInput) {
            gamepadInput = GamepadInput(opMode.gamepad1,opMode.gamepad2)
        }

        val job = GlobalScope.launch {
            opMode.runOpMode()
        }

        try {
            val internal = Class.forName("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal")
            internal.getDeclaredField("isStarted").let {
                it.isAccessible = true
                it.set(opMode,true)
            }

            val stopRequested = internal.getDeclaredField("stopRequested")
            stopRequested.isAccessible = true

            while (!job.isCompleted && (opMode.io?.time() ?: 0.s) < maxTime) {
//                println(opMode.io!!.time())
//                if(opMode.io!!.time() > 2.s) {
//                    (opMode.io as SimIO).triggerDistance = true
//                    println("DID IT??")
//                }
                if(useGamepadInput) {
                    gamepadInput!!.update()
                    Thread.sleep(5)
                } else {
                    Thread.sleep(1000)
                }

//                if(Clock.seconds>1.0) (opMode.io as SimIO).triggerDistance = true
            }

            stopRequested.set(opMode,true)
        } catch (e: Exception) {
            runBlocking { job.cancelAndJoin() }
            println(e)
            throw e
        } finally {
            keyListener?.closeListener()
        }
    }
}
