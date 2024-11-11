package sigmacorns.common.sim

import kotlin.reflect.KClass

interface SigmaHardwareMap {


    fun <T: Any> get(clazz: KClass<T>, deviceName: String?): T {

    }
}