package sigmacorns.constants

import eu.sirotin.kotunil.base.cm

object Color {
    var DIST_THRESHOLD = 5.cm

    fun color(r: Int, g: Int, b: Int): SampleColors? {
        return null
    }
}

enum class SampleColors {
    RED,
    BLUE,
    YELLOW
}