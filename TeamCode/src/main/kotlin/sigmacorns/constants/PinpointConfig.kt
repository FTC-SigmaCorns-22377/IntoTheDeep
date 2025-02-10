package sigmacorns.constants

import com.acmerobotics.dashboard.config.Config
import eu.sirotin.kotunil.base.mm

@Config
object PinpointConfig {
    @JvmField var flipX: Boolean = true
    @JvmField var flipY: Boolean = false
    @JvmField var xOffset: Double = 105.67677.mm.value
    @JvmField var yOffset: Double = 0.mm.value
}