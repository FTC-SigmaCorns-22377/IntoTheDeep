package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math.clamp
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import java.lang.Double.min
import kotlin.math.absoluteValue

class TrapezoidalProfile(val maxSpeed: Expression, val acceleration: Expression) {
    fun new(start: Expression, end: Expression): (Second) -> Expression {
        val delta = (start-end).map { it.absoluteValue }
        val flipped = start>end

        val speed = (acceleration * delta).pow(1.0/2.0).map { min(it,maxSpeed.value) }
        val tToMaxSpeed = speed/acceleration
        val posCoveredDuringAcc = (speed * tToMaxSpeed)/2.0
        val posCoveredAtConstSpeed = delta - posCoveredDuringAcc*2
        val startDecT = tToMaxSpeed + posCoveredAtConstSpeed/speed
        val endT = startDecT + tToMaxSpeed

        return { t ->
            val t = clamp(0.0,endT.value,t.value).s
            val flippedMul = if(flipped) -1.0 else 1.0
            if(t<tToMaxSpeed)
                start + (1.0/2.0*acceleration*(t*t))*flippedMul
            else if(t<startDecT)
                start + (posCoveredDuringAcc + posCoveredAtConstSpeed + speed*(t-startDecT))*flippedMul
            else
                start + (posCoveredDuringAcc + posCoveredAtConstSpeed + speed*(t-startDecT) - 1.0/2.0*acceleration*(t - startDecT).pow(2))*flippedMul
        }
    }
}
