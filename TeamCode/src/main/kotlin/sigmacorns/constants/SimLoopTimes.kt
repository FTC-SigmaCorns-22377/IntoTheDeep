package sigmacorns.constants

import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.ns

object SimLoopTimes {
    val uncertainty: Double = 0.2
    val motorWrite = 5.ms
    val servoWrite = 5.ms
    val pinpointRead = 5.ms
    val base = 10.ns
}