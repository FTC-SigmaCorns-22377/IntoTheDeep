package sigmacorns.common.swerve

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.polar
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.archetypes.Points2D
import net.unnamedrobotics.lib.rerun.rerun
import org.junit.Test
import sigmacorns.common.subsystems.swerve.FlippingSlewRateLimiter
import kotlin.math.PI

class FlippingSlewRateLimiterTest {
    @Test
    fun test() {
        val rr = RerunConnection("FlippingSlewRateLimiterTest","127.0.0.1")
        val controller = FlippingSlewRateLimiter(0.3,0.0)
        val n = 1000

        for(o in 0..n) {
            val pos = PI*2*(o.toDouble()/n.toDouble())
            val points = arrayListOf<Vector2>()
            for (i in 0..n) {
                val theta = PI*2*(i.toDouble()/n.toDouble())
                val limited = controller.updateStateless(1.0,pos, theta)
                points.add(polar(1.m,limited.rad))
            }

            rerun(rr) {
                log("points") {
                    Points2D(
                        origins = points
                    )
                }
            }
        }

        rr.close()
    }
}