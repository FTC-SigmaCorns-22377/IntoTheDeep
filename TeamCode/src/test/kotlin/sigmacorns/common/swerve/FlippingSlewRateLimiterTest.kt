package sigmacorns.common.swerve

import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.rerun.RerunConnection
import org.junit.Before
import org.junit.Test
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
                points.add(Vector2.fromAngle(limited,1.0))
            }

            rr.log("points", points)
        }

        rr.destroy()
    }
}