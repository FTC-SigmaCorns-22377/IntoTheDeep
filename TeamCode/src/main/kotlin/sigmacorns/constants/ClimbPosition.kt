package sigmacorns.constants

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.mm

enum class ClimbPosition(val lift: Metre) {
    FIRST_RUNG(370.mm),
    SECOND_RUNG(700.mm)
}