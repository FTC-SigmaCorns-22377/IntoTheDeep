package sigmacorns.common.simutil

import com.qualcomm.robotcore.hardware.Gamepad
import eu.sirotin.kotunil.base.Second
import kotlinx.coroutines.delay
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.then
import sigmacorns.common.cmd.wait
import kotlin.reflect.KMutableProperty1

fun mockPress(gamepad: Gamepad, button: KMutableProperty1<Gamepad,Boolean>,pressTime: Second)
    = instant { button.set(gamepad,true) } then wait(pressTime) then instant { button.set(gamepad,false) }

fun mockToggle(gamepad: Gamepad, button: KMutableProperty1<Gamepad, Boolean>)
    = instant { button.set(gamepad,!button.get(gamepad)) }

fun instant(f: ()->Unit) = cmd {instant(f)}


/*

 */