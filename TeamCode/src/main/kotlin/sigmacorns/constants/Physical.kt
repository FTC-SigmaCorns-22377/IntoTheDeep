package sigmacorns.constants

import eu.sirotin.kotunil.base.Kilogram
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.revolution
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z
import kotlin.math.pow

object Physical {
    val MOTOR_TICKS_PER_REV = 28.0.tick/ revolution
    val LIFT_GEAR_RATIO = (1.0+(46.0/17.0)).pow(2)
    val INTAKE_GEAR_RATIO = (1.0+(46.0/17.0)).pow(2)/2.0

    val INTAKE_SPOOL_RADIUS = 35.90071.mm/2.0
    val LIFT_SPOOL_RADIUS = 20.mm

//    val EXTEND_M_PER_TICK = (INTAKE_GEAR_RATIO.pow(-1.0)/MOTOR_TICKS_PER_REV*INTAKE_SPOOL_RADIUS).cast(m/ tick)
    val EXTEND_M_PER_TICK = (MOTOR_TICKS_PER_REV.pow(-1) / INTAKE_GEAR_RATIO * INTAKE_SPOOL_RADIUS ).cast(m/ tick)
    val LIFT_M_PER_TICK = (LIFT_GEAR_RATIO.pow(-1.0)/MOTOR_TICKS_PER_REV*LIFT_SPOOL_RADIUS).cast(m/tick)

    // TODO: UPDATE WITH NEW CAD
    val ARM_AXLE_POS: Vector3 = vec3(0.m, (-60).mm,0.m)
    val LIFT_ANGLE: Radian = 90.degrees
    val ARM_LENGTH: Metre = 156.mm
    val CLAW_LENGTH: Metre = 90.mm // NOT EXACT

    // TODO: UPDATE
    // position of the center of the intake when it is straight up relative to the end of the slides
    val INTAKE_CENTER_POS: Vector3 = vec3(72.16213.mm,0.mm,85.18402.mm)

    val WHEEL_RADIUS = 4.8.cm
    val DRIVEBASE_SIZE: Vector3 = vec3(347.17500.mm,284.70000.mm,124.mm)
    val WEIGHT: Kilogram = 20.kg
    val WHEEL_INERTIA: Expression = 0.5*kg*mm*mm
    val DRIVE_RATIO: Double = 13.7
}