package sigmacorns.constants

import com.acmerobotics.dashboard.config.Config
import eu.sirotin.kotunil.base.mm

@Config
object PinpointConfig {
    @JvmField var flipX: Boolean = false
    @JvmField var flipY: Boolean = true
    @JvmField var xOffset: Double = 0.mm.value
    @JvmField var yOffset: Double = (216.3+16.696).mm.value
}