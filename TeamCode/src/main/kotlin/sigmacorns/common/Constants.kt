package sigmacorns.common

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.centimetre
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.H
import eu.sirotin.kotunil.derived.N
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.derived.Ω
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inches
import net.unnamedrobotics.lib.math2.revolution
import net.unnamedrobotics.lib.math2.revolutions
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.vec3
import kotlin.math.hypot

object Constants {

    val ANALOG_MAX = 3.3.V

    /*-----PHYSICAL MEASUREMENTS-----*/

    /**
     * X is the length of the side plates
     */
    val DRIVEBASE_SIZE = vec3(329.59376.mm,335.94376.mm,65.52500.mm)


    val ODO_CENTER = vec3()

    /**
     * Position of the axle relative to the center of the robot.
     */
    val AXLE_CENTER = vec3((-127.49760).mm,0.mm,142.356.mm)

    /**
     * The radius of the axle when visualized through rerun.
     */
    val AXLE_VISUAL_RADIUS = 10.mm

    val BOXTUBE_1_VISUAL_OFFSET = 32.8.mm
    val BOXTUBE_SECTION_LENGTH = 330.mm
    val BOXTUBE_VISUAL_SIZES = listOf(1.5.inches,1.inches,0.5.inches)

    /**
     * Orthogonal distance between the box tube and the arm axle
     */
    val ARM_OFFSET = 66.13235.mm

    /**
     * Length between the pivot axis of the claw and the ideal gripped sample position
     */
    val CLAW_LENGTH = 0.m

    val ARM_MOTOR_GEAR_RATIO = (((1+(46/11))) * (1+(46/11))).rad / rad

    // -----GoBilda Motor Constants-----
    /**
     * Encoder ticks per internal revolution of each motor
     */
    val MOTOR_TICK_RATIO = 28.tick / revolution
    val MOTOR_TORQUE_CONSTANT = 0.0188605*(N*m / A)
    val MOTOR_INDUCTANCE = 0.01*H
    val MOTOR_RESISTANCE = 1.15674*Ω

    /**
     * arm pulley ratio = 16:36
     * gearing between pulley and spool = 80:36,
     * spool radius = 25mm
     * bc its cascade the arm extends 2m for every m of string spooled.
     */
    val ARM_PULLEY_RATIO = 16.revolutions / 36.revolutions
    val ARM_DIFFY_RATIO = 80.revolutions / 36.revolutions
    val ARM_SPOOL_RADIUS = 25.mm

    val ARM_PIVOT_RATIO = (ARM_PULLEY_RATIO/ARM_MOTOR_GEAR_RATIO/MOTOR_TICK_RATIO).cast(revolution/tick)
    val ARM_EXTENSION_RATIO = (ARM_PIVOT_RATIO * ( ARM_DIFFY_RATIO * ARM_SPOOL_RADIUS*revolution / revolution) * 2.m/m).cast(m/tick)

    val ARM_MOMENT_RATIO =
        ((78321.89077 * (kg*mm*mm) - 20598.01367 * (kg*mm*mm))
                / (871.58165.mm - 369.90937.mm))
        .cast(kg*m*m / m)
    val ARM_COM_DIST_TO_PIVOT = hypot(68.58108,18.61353).mm

    /**
     * Ratio between pitch and roll axes of the claw.
     */
    val CLAW_PITCH_RATIO = 1.rad / 1.rad

    val CLAW_ROLL_RATIO = 1.rad / 1.rad


    /*-----BOUNDS-----*/
    val ARM_PIVOT_BOUNDS: Bounds<Radian> = Bounds(10.degrees,90.degrees)
    val ARM_EXTENSION_BOUNDS: Bounds<Metre> = Bounds(376.43997.mm,871.58165.mm)
    val CLAW_SERVO_1_BOUNDS: Bounds<Radian> = Bounds((-180).degrees, (-180+355.0).degrees)
    val CLAW_SERVO_2_BOUNDS: Bounds<Radian> = Bounds((-180).degrees, (-180+355.0).degrees)

    /**-----SAFETY-----*/

    /**
     * How far away the extension needs to be from [ARM_EXTENSION_BOUNDS] to run full power.
     *
     * When the current position is within the threshold the power is clamped to [ARM_SAFE_EXTENSION_POWER].
     */
    val ARM_SAFE_EXTENSION_THRESH: Metre = 10.centimetre


    /**
     * How far away the pivot needs to be from [ARM_PIVOT_BOUNDS] to run full power.
     *
     * When the distance between the pivot bounds is less than it, the power is clamped to [ARM_SAFE_PIVOT_POWER].
     */
    val ARM_SAFE_PIVOT_THRESH: Radian = 10.degrees

    /**
     * The maximum power to apply to the extension when it is under [ARM_SAFE_EXTENSION_THRESH]
     */
    val ARM_SAFE_EXTENSION_POWER = 1.V

    /**
     * The maximum power to apply to the pivot when it is under [ARM_SAFE_PIVOT_THRESH]
     */
    val ARM_SAFE_PIVOT_POWER = 2.V

    /**
     * The maximum power to apply to an individual arm motor.
     */
    val ARM_MAX_MOTOR_POWER = 12.V

    val CLAW_CLOSED = 1.0
    val CLAW_OPEN = 0.0
}

object Tuning {
    val ARM_PIVOT_PID = PIDCoefficients(0.001,0.0,0.00)
    val ARM_EXTENSION_PID = PIDCoefficients(0.0,0.0,0.0)
    val SWERVE_MODULE_PID = PIDCoefficients(0.00004,0.0,0.0)
}