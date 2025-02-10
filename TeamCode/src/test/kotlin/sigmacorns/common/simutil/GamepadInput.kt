package sigmacorns.common.simutil

import com.github.strikerx3.jxinput.XInputDevice
import com.qualcomm.robotcore.hardware.Gamepad

class GamepadInput(val gamepad1: Gamepad, val gamepad2: Gamepad) {
    val devices = XInputDevice.getAllDevices()

    var x1: XInputDevice? = null
    var x2: XInputDevice? = null

    fun update() {
        devices.filter { it.isConnected }.forEach {
            if(it.poll()) {
//                println("device: ${it.playerNum}")
                val components = it.components
                val buttons = components.buttons
                if(buttons.a && buttons.start) {
                    x1 = it
                    if(it==x2) x2 = null
                } else if(buttons.b && buttons.start) {
                    x2 = it
                    if(it==x1) x1 = null
                }

                val curGamepad = if(it==x1) gamepad1 else if(it==x2) gamepad2 else null
//                println("gamepad1a = ${gamepad1.a}, gamepad2a: ${gamepad2.a}")
                if(curGamepad!=null) {
                    curGamepad.a = buttons.a
                    curGamepad.b = buttons.b
                    curGamepad.x = buttons.x
                    curGamepad.y = buttons.y

                    curGamepad.start = buttons.start
                    curGamepad.back = buttons.back
                    curGamepad.guide = buttons.guide

                    curGamepad.dpad_up = buttons.up
                    curGamepad.dpad_down = buttons.down
                    curGamepad.dpad_left = buttons.left
                    curGamepad.dpad_right = buttons.right

                    curGamepad.left_stick_button = buttons.lThumb
                    curGamepad.right_stick_button = buttons.rThumb
                    curGamepad.left_bumper = buttons.lShoulder
                    curGamepad.right_bumper = buttons.rShoulder

                    val axes = components.axes

                    curGamepad.left_stick_x = axes.lx
                    curGamepad.left_stick_y = axes.ly
                    curGamepad.right_stick_x = axes.rx
                    curGamepad.right_stick_y = axes.ry

                    curGamepad.left_trigger = axes.lt
                    curGamepad.right_trigger = axes.rt
                }

            }
        }
    }
}