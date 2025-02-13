package sigmacorns.common.control

import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyKinematics
import sigmacorns.common.kinematics.DiffyOutputPose
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

class PIDDiffyController(
    private val kinematics: DiffyKinematics,
    private var axis1PIDCoefficients: PIDCoefficients,
    private var axis2PIDCoefficients: PIDCoefficients,
    private val axis1Bounds: Bounds<Expression>,
    private val axis2Bounds: Bounds<Expression>,
    private val axis1SafePowerTheshold: Expression,
    private val axis2SafePowerTheshold: Expression,
    private val axis1SafePower: Volt,
    private val axis2SafePower: Volt,
    private val motorMaxPower: Volt
): Controller<DiffyInputPose, List<Volt>, DiffyOutputPose>() {
    val pid1 = PIDController(axis1PIDCoefficients)
    val pid2 = PIDController(axis2PIDCoefficients)

    override lateinit var output: List<Volt>
    override lateinit var position: DiffyInputPose
    override lateinit var target: DiffyOutputPose

    override fun copy(): Controller<DiffyInputPose, List<Volt>, DiffyOutputPose> {
        TODO("Not yet implemented")
    }

    override fun update(dt: Double): List<Volt> {
        val x = position
        val t = target
        pid1.coefficients = axis1PIDCoefficients
        pid2.coefficients = axis2PIDCoefficients

        val tBounded = DiffyOutputPose(
            axis1Bounds.apply(t.axis1),
            axis2Bounds.apply(t.axis2)
        )

        return kinematics.forward(x).let {
            if((tBounded.axis1-it.axis1).value.sign != pid1.previousError.sign) pid1.integral = 0.0
            if((tBounded.axis2-it.axis2).value.sign != pid2.previousError.sign) pid2.integral = 0.0

            var axis1Power = pid1.updateStateless(dt,it.axis1.value,tBounded.axis1.value).V
            var axis2Power = pid2.updateStateless(dt,it.axis2.value,tBounded.axis2.value).V

//            println("Axis 1 raw: $axis1Power, Axis 2 raw: $axis2Power. target: ${t.axis1},${t.axis2}")

            val axis1OverMax = it.axis1 > axis1Bounds.max - axis1SafePowerTheshold
            val axis1UnderMin = it.axis1 < axis1Bounds.min + axis1SafePowerTheshold
            val axis2OverMax = it.axis2 > axis2Bounds.max - axis2SafePowerTheshold
            val axis2UnderMin = it.axis2 < axis2Bounds.min + axis2SafePowerTheshold

            val axis1PowerBounds = Bounds(
                if(axis1UnderMin) axis1SafePower.map { v -> -v } else (-Double.MAX_VALUE).V,
                if(axis1OverMax) axis1SafePower else Double.MAX_VALUE.V
            )

            val axis2PowerBounds = Bounds(
                if(axis2UnderMin) axis2SafePower.map { v -> -v } else (-Double.MAX_VALUE).V,
                if(axis2OverMax) axis2SafePower else Double.MAX_VALUE.V
            )

            axis1Power = axis1PowerBounds.apply(axis1Power)
            axis2Power = axis2PowerBounds.apply(axis2Power)

            val powers = listOf(axis1Power+axis2Power,axis1Power-axis2Power)

            val scalar = min(1.0, motorMaxPower.value/powers.maxOfOrNull { abs(it.value) }!! )
            powers.map { p -> (p*scalar).cast(V) }
        }
    }
}

