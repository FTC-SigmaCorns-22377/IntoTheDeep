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

object Limits {
    val EXTENSION: Bounds<Metre> = Bounds((-15).mm,600.mm)
    val LIFT: Bounds<Metre> = Bounds(0.mm,670.mm)

    val EXTENSION_SAFE_THRESH: Metre = 5.cm
    val EXTENSION_SAFE_POWER: Volt = 12.V

    val LIFT_SAFE_THRESH: Metre = 5.cm
    val LIFT_SAFE_POWER: Volt = 8.V

    val SLIDE_MOTOR_MAX: Volt = 12.V

    private val ARM_SERVO_RANGE = 355.degrees

    // 1 = 0.37,   2 = 0.58 is straight up.
    // zero = -ARM_SERVO_RANGE*0.37
    // zero2 = -ARM_SERVO_RANGE*0.58
    private val zero1 = -ARM_SERVO_RANGE*0.5
    private val zero2 = -ARM_SERVO_RANGE*0.5
    val ARM_SERVO_1: Bounds<Radian> = Bounds((zero1).cast(rad), ((zero1) + ARM_SERVO_RANGE).cast(rad))
    val ARM_SERVO_2: Bounds<Radian> = Bounds((zero2 ).cast(rad), ((zero2)+ ARM_SERVO_RANGE).cast(rad))

    private val WRIST_SERVO_RANGE = 355.degrees

    private val zeroWrist = -WRIST_SERVO_RANGE*0.29 - 2.16.rad
    val WRIST_SERVO: Bounds<Radian> = Bounds((zeroWrist).cast(rad), ((zeroWrist) + WRIST_SERVO_RANGE).cast(rad))
}