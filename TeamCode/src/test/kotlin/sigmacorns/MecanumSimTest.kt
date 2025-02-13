package sigmacorns

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import org.junit.Test
import sigmacorns.common.sim.SimNative

class MecanumSimTest {
    @Test
    fun test() {
        val sim = SimNative(0.0,0.0, Transform2D(0.m,0.m,0.rad))


    }
}