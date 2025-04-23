package sigmacorns.constants

import com.acmerobotics.dashboard.config.Config
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.core.Expression

@Config
object Color {
    @JvmField var DIST_THRESHOLD_CM = 2.5

    fun color(dist: Expression, r: Int, g: Int, b: Int): SampleColors? {
        return when {
            dist.value*100>DIST_THRESHOLD_CM -> null
            (g > b && g > r) || g+b+r >= 12 -> SampleColors.YELLOW
            b > r && b > g -> SampleColors.BLUE
            else -> SampleColors.RED
        }
    }
}

enum class SampleColors {
    RED,
    BLUE,
    YELLOW
}