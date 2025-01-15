package sigmacorns.common.simutil

import com.github.kwhat.jnativehook.GlobalScreen
import com.github.kwhat.jnativehook.NativeHookException
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent.*
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener
import com.qualcomm.robotcore.hardware.Gamepad
import net.unnamedrobotics.lib.util.Clock

class KeyboardGamepads(val gm1: Gamepad, val gm2: Gamepad): NativeKeyListener {
    var sendingToGamepad1: Boolean = true
    val STICK_SPEED = 2

    var lastTime: Float = Clock.seconds.toFloat()
    var dt: Float = 0f

    val loggedKeys = setOf(
        // dpad
        VC_UP,
        VC_DOWN,
        VC_LEFT,
        VC_RIGHT,

        // left stick
        VC_I,
        VC_J,
        VC_K,
        VC_L,

        // buttons
        VC_A,
        VC_B,
        VC_X,
        VC_Y
    )

    val pressedKeys: MutableSet<Int> = mutableSetOf()

    override fun nativeKeyPressed(nativeEvent: NativeKeyEvent) {
        val code = nativeEvent.keyCode
        if(code == VC_TAB) sendingToGamepad1 = !sendingToGamepad1
        if(code in loggedKeys) pressedKeys.add(code)

        if (code == VC_ESCAPE) {
            try {
                GlobalScreen.unregisterNativeHook()
            } catch (nativeHookException: NativeHookException) {
                nativeHookException.printStackTrace()
            }
        }

        updateGamepad()
    }

    fun startListener() {
        GlobalScreen.registerNativeHook()
        GlobalScreen.addNativeKeyListener(this)
    }

    fun closeListener() {
        GlobalScreen.unregisterNativeHook()
    }

    override fun nativeKeyReleased(nativeEvent: NativeKeyEvent) {
        val code = nativeEvent.keyCode
        if(code in loggedKeys) pressedKeys.remove(code)
        updateGamepad()
    }

    private fun updateGamepad() {
        val t = Clock.seconds.toFloat()
        dt = t-lastTime
        lastTime = t

        val curGamepad = if(sendingToGamepad1) gm1 else gm2

        curGamepad.a = pressedKeys.contains(VC_A)
        curGamepad.b = pressedKeys.contains(VC_B)
        curGamepad.x = pressedKeys.contains(VC_X)
        curGamepad.y = pressedKeys.contains(VC_Y)

        curGamepad.dpad_up = pressedKeys.contains(VC_UP)
        curGamepad.dpad_down = pressedKeys.contains(VC_DOWN)
        curGamepad.dpad_left = pressedKeys.contains(VC_LEFT)
        curGamepad.dpad_right = pressedKeys.contains(VC_RIGHT)

        curGamepad.left_stick_y = updateStickAxis(VC_K,VC_I,curGamepad.left_stick_y)
        curGamepad.left_stick_x = updateStickAxis(VC_L,VC_J,curGamepad.left_stick_x)
    }

    private fun updateStickAxis(upKey: Int, downKey: Int, cur: Float): Float {
        var cur = cur

        val u = pressedKeys.contains(upKey)
        val d = pressedKeys.contains(downKey)

        if(!(u || d)) return 0f
        if(u) cur += STICK_SPEED*dt
        if(d) cur -= STICK_SPEED*dt

        return cur
    }
}