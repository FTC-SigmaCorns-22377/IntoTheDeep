package sigmacorns.constants

import sigmacorns.constants.Tuning.FLAP_CLOSED
import sigmacorns.constants.Tuning.FLAP_EJECT
import sigmacorns.constants.Tuning.FLAP_OPEN

enum class FlapPosition(val x: Double) {
    CLOSED(FLAP_CLOSED),
    EJECT(FLAP_EJECT),
    OPEN(FLAP_OPEN)
}
