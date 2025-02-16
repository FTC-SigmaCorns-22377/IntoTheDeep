package sigmacorns.constants

import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad

enum class IntakePosition(val x: Radian) {
    OVER(0.rad),
    BACK((-0.45).rad),
    ACTIVE((-0.00).rad)
}