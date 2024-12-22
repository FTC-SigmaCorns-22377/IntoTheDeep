package sigmacorns.common.subsystems.swerve

import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.SimpleController
import net.unnamedrobotics.lib.math.clamp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import kotlin.math.PI
import kotlin.math.absoluteValue

class FlippingSlewRateLimiter(val maxRate: Double, val flipCooldown: Double): SimpleController<Double>() {
    override var output: Double = 0.0
    override var position: Double = 0.0
    override var target: Double = 0.0

    override fun update(deltaTime: Double): Double {
        val maxErr = deltaTime*maxRate
        val errStraight = normalizeRadians(target - position)
        val errFlipped = normalizeRadians(errStraight + PI)
        val predicate = errFlipped.absoluteValue<errStraight.absoluteValue
        val p = if(predicate) position+PI else position
        val e = if(predicate) errFlipped else errStraight

        return normalizeRadians(p + clamp(-maxErr,e,maxErr))
    }

    override fun copy(): Controller<Double, Double, Double> = FlippingSlewRateLimiter(maxRate,flipCooldown)
}