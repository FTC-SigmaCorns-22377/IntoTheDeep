package sigmacorns

import net.unnamedrobotics.lib.gamepad.Button
import org.junit.Test

class ButtonTest {
    @Test
    fun test() {
        var a = false
        val button = Button { a }
        println("${button.isJustPressed}")
        button.periodic()
        println("${button.isJustPressed}")
        a = true
        button.periodic()
        println("${button.isJustPressed}")
        button.periodic()
        println("${button.isJustPressed}")

    }
}