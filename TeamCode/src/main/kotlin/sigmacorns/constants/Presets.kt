package sigmacorns.constants

import eu.sirotin.kotunil.base.cm
import net.unnamedrobotics.lib.math2.inches
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z

object AutoSamples {
    val SAMPLE_Y_1 = 60.cm -1.inches
    val SAMPLE_X = 120.cm - 2.inches
    val SAMPLE_HEIGHT = 1.5.inches

    val BLUE_BUCKET_SAMPLE_1 = vec3(24.inches*3 - SAMPLE_X, SAMPLE_Y_1, SAMPLE_HEIGHT /2.0)
    val BLUE_BUCKET_SAMPLE_2 = vec3(BLUE_BUCKET_SAMPLE_1.x, BLUE_BUCKET_SAMPLE_1.y + 10.inches, BLUE_BUCKET_SAMPLE_1.z)
    val BLUE_BUCKET_SAMPLE_3 = vec3(BLUE_BUCKET_SAMPLE_2.x, BLUE_BUCKET_SAMPLE_2.y + 10.inches, BLUE_BUCKET_SAMPLE_2.z)

//    val INTAKE_
}