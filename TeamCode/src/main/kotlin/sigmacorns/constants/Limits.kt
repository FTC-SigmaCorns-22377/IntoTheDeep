package sigmacorns.constants

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.mapRanges

object Limits {
    val EXTENSION: Bounds<Metre> = Bounds((-10).mm,600.mm)
    val LIFT: Bounds<Metre> = Bounds(0.mm,770.mm)

    val EXTENSION_SAFE_THRESH: Metre = 5.cm
    val EXTENSION_SAFE_POWER: Volt = 8.V

    val LIFT_SAFE_THRESH: Metre = 5.cm
    val LIFT_SAFE_POWER: Volt = 8.V

    val SLIDE_MOTOR_MAX: Volt = 12.V

    private val ARM_SERVO_RANGE = 355.degrees*40.0/48.0

    // 1 = 0.37,   2 = 0.58 is straight up.
    // zero = -ARM_SERVO_RANGE*0.37
    // zero2 = -ARM_SERVO_RANGE*0.58
    private val zero1 = -ARM_SERVO_RANGE*0.62
    private val zero2 = -ARM_SERVO_RANGE*0.31
    val ARM_SERVO_1: Bounds<Radian> = Bounds((zero1).cast(rad), ((zero1) + ARM_SERVO_RANGE).cast(rad))
    val ARM_SERVO_2: Bounds<Radian> = Bounds((zero2 ).cast(rad), ((zero2)+ ARM_SERVO_RANGE).cast(rad))

    val INTAKE_SERVO_1: Bounds<Radian> = Bounds((180-355).degrees,180.degrees)
    val INTAKE_SERVO_2: Bounds<Radian> = Bounds((180-355).degrees,180.degrees)
}

fun Bounds<Radian>.toServoPos(): (Radian)->Double
= { mapRanges(min.value..max.value,0.0..1.0)(it.value).toDouble() }