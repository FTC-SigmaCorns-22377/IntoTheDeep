package sigmacorns.common.sim

import com.qualcomm.robotcore.hardware.HardwareMap

class PhysicalHardwareMap(val hardwareMap: HardwareMap): SigmaHardwareMap {
    override fun <T> get(clazz: Class<out T>?, deviceName: String?): T {
        
    }
}