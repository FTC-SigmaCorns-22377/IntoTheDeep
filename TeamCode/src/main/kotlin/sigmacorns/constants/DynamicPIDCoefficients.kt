package sigmacorns.constants

import com.acmerobotics.dashboard.config.Config
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients

@Config
object DynamicPIDCoefficients1 {
    @JvmField var P: Double = 12.0
    @JvmField var I: Double = 0.0
    @JvmField var D: Double = 1.5

    fun coeff() = PIDCoefficients(P,I,D)
}

@Config
object DynamicPIDCoefficients2 {
    @JvmField var P: Double = 0.0
    @JvmField var I: Double = 0.0
    @JvmField var D: Double = 0.0

    fun coeff() = PIDCoefficients(P, I, D)
}

@Config
object DynamicFFCoefficients {
    @JvmField var kv: Double = 0.5
    @JvmField var ka: Double = 0.5
}