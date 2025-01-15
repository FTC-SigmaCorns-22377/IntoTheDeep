package sigmacorns.common

import com.acmerobotics.dashboard.config.Config
import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.centimetre
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.ns
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.H
import eu.sirotin.kotunil.derived.Hz
import eu.sirotin.kotunil.derived.N
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.derived.Ω
import eu.sirotin.kotunil.specialunits.g
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inches
import net.unnamedrobotics.lib.math2.revolution
import net.unnamedrobotics.lib.math2.revolutions
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import sigmacorns.common.subsystems.arm.TrapezoidalProfile
import kotlin.math.hypot
import kotlin.reflect.KProperty

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
    val CLAW_LENGTH = 5.5.inches

    val ARM_MOTOR_GEAR_RATIO = (((1+(46/11))) * (1+(46/11))).rad / rad

    /**
     * Limelight Pose3d in robot coordinates; ROT contains (yaw, pitch, roll).
     * I avoided vec3 since the components kept returning as expressions and I'm too lazy to fix it.
     * TODO switch back to vec3 when time is less urgent.
     */
    val LIMELIGHT_OFFSET_VEC = arrayOf(0.0,0.0,0.0)
    val LIMELIGHT_OFFSET_ROT = arrayOf(0.0,0.0,0.0)

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
        ((78321.89077 * (g*mm*mm) - 20598.01367 * (g*mm*mm))
                / (871.58165.mm - 369.90937.mm))
        .cast(kg*m*m / m)
    val ARM_COM_DIST_TO_PIVOT = hypot(68.58108,18.61353).mm

    /**
     * Ratio between pitch and roll axes of the claw.
     */
    val CLAW_PITCH_RATIO = 1.rad / 1.rad

    val CLAW_ROLL_RATIO = 1.rad / 1.rad

    // -----Camera Calibration-----
    /**
     * Sigma usually calibrates with OpenCV, but I tried Limelight's default ChArUco.
     * I calibrated at the highest resolution of 1280x960 40fps with full exposure (3300),
     * half sensor gain (22.5/45), and around half red and blue balance (1500/2500).
     * Out of the 100 snapshots I took, 42 were deemed suitable for calibration.
     */
    val REPROJECTION_ERROR = 0.9
    val PRINCIPAL_PIXEL_OFFSET = vec2(7.801,20.489) // default (-2.774,22.549)

    /**
     * Field of View, stored in the formats (HORIZONTAL_TOTAL, LEFT_TO_CENTER, CENTER_TO_RIGHT) and
     * (VERTICAL_TOTAL, TOP_TO_CENTER, CENTER_TO_BOTTOM). Note that the first value is the sum of
     * the second and third values.
     */
    val HORIZONTAL_FOV = vec3(54.621.degrees,27.571.degrees,27.050.degrees) // default (54.505,27.163,27.342)
    val VERTICAL_FOV = vec3(42.448.degrees,22.074.degrees,20.374.degrees) // default (42.239,22.069,20.170)

    /**
     * Distortion coefficients derived from the Brown-Conrady model, stored as (K1,K2,P1,P2,K3)
     * according to FIRST tradition. Here Kn and Pn are respectively the nth radial and tangential
     * distortion coefficients (so K1 is the quadratic coeff., K2 the quartic, K3 the sextic, etc).
     */
    val DISTORTION_COEFFICIENTS = arrayOf(0.215551,-0.770350,-0.000670,0.001687,0.923038)
    // default (0.177168,-0.457341,0.000360,0.002753,0.178259)

    /**
     * Camera matrix, storing focal lengths (fx,fy) at entries (1,1) and (2,2) and optical center
     * (cx,cy) at entries (1,3) and (2,3). Remark that (cx,cy) - [PRINCIPAL_PIXEL_OFFSET] = (1280/2,960/2),
     * the theoretical optical center for the resolution in question (1280x960).
     */
    val CAMERA_MATRIX = arrayOf(arrayOf(1215.838,0.0,647.801),arrayOf(0.0,1215.106,500.489),arrayOf(0.0,0.0,1.0))
    // default ((1221.445,0.0,637.226),(0.0,1223.398,502.549),(0,0,1))


    /*-----BOUNDS-----*/
    val ARM_PIVOT_BOUNDS: Bounds<Radian> = Bounds((-30).degrees,90.degrees)
    val ARM_EXTENSION_BOUNDS: Bounds<Metre> = Bounds(376.43997.mm,1010.58165.mm)
    val CLAW_SERVO_1_BOUNDS: Bounds<Radian> = Bounds((-180).degrees, (-180+355.0).degrees)
    val CLAW_SERVO_2_BOUNDS: Bounds<Radian> = Bounds((-180).degrees, (-180+355.0).degrees)
    val CLAW_SERVO_3_BOUNDS: Bounds<Radian> = Bounds((-180).degrees, (-180+355.0).degrees) //for open/close servo

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
    val ARM_SAFE_EXTENSION_POWER = 3.V

    /**
     * The maximum power to apply to the pivot when it is under [ARM_SAFE_PIVOT_THRESH]
     */
    val ARM_SAFE_PIVOT_POWER = 4.V

    /**
     * The maximum power to apply to an individual arm motor.
     */
    val ARM_MAX_MOTOR_POWER = 12.V

    val CLAW_CLOSED = 0.6
    val CLAW_OPEN = 0.3

    val MODULE_OFFSET = arrayOf(
        0.7197103170042072.rad,
        3.4881198432584855.rad,
        4.188790204786391.rad,
        3.383400088138826.rad,
    )
}

@Config
object Tuning {
    val ARM_G = (-3.5).V
//    val ARM_PIVOT_PID = PIDCoefficients(9.0,0.000000,-0.9)

    val ARM_PIVOT_PROFILE = TrapezoidalProfile(2.rad/s,2.rad/s/s)
    val ARM_PROFILE_DIST = 10.degrees

    @JvmField
    var ARM_PIVOT_STATIC = 0.0
    @JvmField
    var ARM_PIVOT_SATIC_THRESH = 0.08

    val ARM_PIVOT_PID
        get() = tunePID()
    val ARM_EXTENSION_PID = PIDCoefficients(24.0,0.0,0.0)
//        get() = tunePID()
    val SWERVE_MODULE_PID = PIDCoefficients(1.0,0.0,0.0)
//        get() = tunePID()
}

object LoopTimes {
    val SWERVE = 500.Hz
    val ARM = 100.Hz
    val CHOREO = 100.Hz
    val SWERVE_POS_UPDATE = 50.Hz

    const val DRIVE_UPDATE_THRESHOLD = 0.01
    const val TURN_UPDATE_THRESHOLD = 0.02
    const val ARM_UPDATE_THRESHOLD = 0.02
    const val DIFFY_UPDATE_THRESHOLD = 0.005
}

object SimIOTimes {
    const val uncertainty = 0.2
    val bulkRead = 1.ms
    val motorWrite = 1.ms
    val servoWrite = 1.ms
    val pinpointFetch = 2.ms
    val base = 10.ns
}

@Suppress("SimplifyBooleanWithConstants", "MemberVisibilityCanBePrivate", "KotlinConstantConditions" )
object LOGGING {
    const val ALL_LOG: Boolean = true
    const val LOG_IO: Boolean = false && ALL_LOG
    const val RERUN_SWERVE: Boolean = true && ALL_LOG
    const val RERUN_CHOREO: Boolean = true && ALL_LOG
    const val RERUN_ARM: Boolean = true && ALL_LOG
}

@Config
object PIDTune {
    @JvmField
    var P = 20.0
    @JvmField
    var I = 0.0
    @JvmField
    var D = 0.0
}

fun tunePID(): PIDCoefficients = PIDCoefficients(PIDTune.P, PIDTune.I,PIDTune.D)