package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math.clamp
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import java.lang.Double.min

class TrapezoidalProfile(val maxSpeed: Expression, val acceleration: Expression) {
    fun new(start: Expression, end: Expression): (Second) -> Expression {
        var start = start
        var end = end
        val flipped = start>end
        if(flipped) {
            val tmp = start
            start = end
            end = tmp
        }

        val speed = (acceleration * (end-start)).pow(1.0/2.0).map { min(it,maxSpeed.value) }
        val tToMaxSpeed = speed/acceleration
        val posCoveredDuringAcc = (speed * tToMaxSpeed)/2.0
        val posCoveredAtConstSpeed = (end-start) - posCoveredDuringAcc*2
        val startDecT = tToMaxSpeed + posCoveredAtConstSpeed/speed
        val endT = startDecT + tToMaxSpeed

        return { t ->
            var t = clamp(0.0,endT.value,t.value).s
            if(flipped) t = (endT-t).cast(s)
            if(t<tToMaxSpeed) start + 1.0/2.0*acceleration*(t*t)
            else if(t>tToMaxSpeed)
                posCoveredDuringAcc + posCoveredAtConstSpeed + speed*(t-startDecT)
            else
                posCoveredDuringAcc + posCoveredAtConstSpeed + speed*(t-startDecT) - 1.0/2.0*acceleration*(t - startDecT).pow(2)
        }
    }
}
