package sigmacorns.common

import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.s
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.junit.Test
import sigmacorns.common.simutil.KeyboardGamepads
import sigmacorns.opmode.SIM
import sigmacorns.opmode.SimOrHardwareOpMode
import sigmacorns.opmode.Teleop
import sigmacorns.opmode.test.ArmTest
import sigmacorns.opmode.test.ClawMachineTest

class OpModeTest {
    @Test
    fun test() {
        SIM = true

        val opMode: SimOrHardwareOpMode = ClawMachineTest()
        val maxTime = 10.s

        opMode.gamepad1 = Gamepad()
        opMode.gamepad2 = Gamepad()

        val keyListener = KeyboardGamepads(opMode.gamepad1, opMode.gamepad2)
        keyListener.startListener()

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

            while (!job.isCompleted && (opMode.io?.time() ?: 0.s) < maxTime) { Thread.sleep(1000) }

            stopRequested.set(opMode,true)
        } catch (e: Exception) {
            runBlocking { job.cancelAndJoin() }
            println(e)
            throw e
        } finally {
            keyListener.closeListener()
        }
    }
}