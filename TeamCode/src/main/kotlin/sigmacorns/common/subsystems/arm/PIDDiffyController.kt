package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.lerp
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.Constants
import sigmacorns.common.Tuning
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin

class PIDDiffyController(val kinematics: DiffyKinematics, var axis1PIDCoefficients: PIDCoefficients, var axis2PIDCoefficients: PIDCoefficients): Controller<DiffyInputPose, ArmMotorPowers, DiffyOutputPose>() {
    var lastBoundedExtensionPower: Volt = 0.V
    var lastBoundedPivotPower: Volt = 0.V
    var lastRawExtensionPower: Volt = 0.V
    var lastRawPivotPower: Volt = 0.V
    val pid1 = PIDController(axis1PIDCoefficients)
    val pid2 = PIDController(axis2PIDCoefficients)

    override lateinit var output: ArmMotorPowers
    override lateinit var position: DiffyInputPose
    override lateinit var target: DiffyOutputPose

    override fun copy(): Controller<DiffyInputPose, ArmMotorPowers, DiffyOutputPose> {
        TODO("Not yet implemented")
    }

    override fun update(dt: Double): ArmMotorPowers {
        val x = position
        val t = target
        pid1.coefficients = axis1PIDCoefficients
        pid2.coefficients = axis2PIDCoefficients

        val tBounded = DiffyOutputPose(
            Constants.ARM_PIVOT_BOUNDS.apply(t.axis1.cast(rad)),
            Constants.ARM_EXTENSION_BOUNDS.apply(t.axis2.cast(m))
        )

        val powers = kinematics.forward(x).let {
            var pivot = pid1.updateStateless(dt,it.axis1.value,tBounded.axis1.value).V
            var extension = pid2.updateStateless(dt,it.axis2.value,tBounded.axis2.value).V

            if((it.axis1.value-target.axis1.value).absoluteValue < Tuning.ARM_PIVOT_SATIC_THRESH)
                pivot = pivot.map { it + it.sign* Tuning.ARM_PIVOT_STATIC }

            lastRawPivotPower = pivot
            lastRawExtensionPower = extension

            val pivotOverMax = it.axis1 > Constants.ARM_PIVOT_BOUNDS.max - Constants.ARM_SAFE_PIVOT_THRESH
            val pivotUnderMin = it.axis1 < Constants.ARM_PIVOT_BOUNDS.min + Constants.ARM_SAFE_PIVOT_THRESH
            val extensionOverMax = it.axis2 > Constants.ARM_EXTENSION_BOUNDS.max - Constants.ARM_SAFE_EXTENSION_THRESH
            val extensionUnderMin = it.axis2 < Constants.ARM_EXTENSION_BOUNDS.min + Constants.ARM_SAFE_EXTENSION_THRESH

            val pivotPowerBounds = Bounds(
                if(pivotUnderMin) Constants.ARM_SAFE_PIVOT_POWER.map { v -> -v } else (-Double.MAX_VALUE).V,
                if(pivotOverMax) Constants.ARM_SAFE_PIVOT_POWER else Double.MAX_VALUE.V
            )

            val extensionPowerBounds = Bounds(
                if(extensionUnderMin) Constants.ARM_SAFE_EXTENSION_POWER.map { v -> -v } else (-Double.MAX_VALUE).V,
                if(extensionOverMax) Constants.ARM_SAFE_EXTENSION_POWER else Double.MAX_VALUE.V
            )

            val t = Constants.ARM_EXTENSION_BOUNDS.let { b ->
                (it.axis2.value-b.min.value)/(b.max.value-b.min.value)
            }

            val gE = (Tuning.ARM_G_MIN..Tuning.ARM_G_MAX).lerp(t.clampMagnitude(1.0))

            var g = sin(it.axis1.value) *gE.V*it.axis2.value

//                if(it.axis1.value.absoluteValue < target.pivot.value.absoluteValue) g = 0.V
            pivot = pivotPowerBounds.apply((pivot + g).cast(V))
            extension = extensionPowerBounds.apply(extension)


            lastBoundedPivotPower = pivot
            lastBoundedExtensionPower = extension

            val powers = listOf(pivot+extension,pivot-extension)

            val scalar = min(1.0, Constants.ARM_MAX_MOTOR_POWER.value/powers.maxOfOrNull { abs(it.value) }!! )
            powers.map { p -> (p*scalar).cast(V) }
        }

        return powers
    }
}