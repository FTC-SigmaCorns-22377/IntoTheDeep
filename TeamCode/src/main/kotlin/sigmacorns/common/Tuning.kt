package sigmacorns.common

import com.acmerobotics.dashboard.config.Config
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.degrees
import sigmacorns.common.subsystems.arm.TrapezoidalProfile

@Config
object Tuning {
    @JvmField
    var ARM_G_MIN = -1.0

    @JvmField
    var ARM_G_MAX = -3.0

    val ARM_PIVOT_PROFILE_DOWN = TrapezoidalProfile(3.5.rad / s, 4.5.rad / s / s)
    val ARM_PIVOT_PROFILE_UP = TrapezoidalProfile(2.rad / s, 2.3.rad / s / s)

    @JvmField
    var ARM_PROFILE_DIST_DEG = 10

    val ARM_PROFILE_DIST
        get() = ARM_PROFILE_DIST_DEG.degrees

    @JvmField
    var ARM_EXTEND_FINE_DIST_CM = 5


    @JvmField
    var ARM_PIVOT_STATIC = 0.0
    @JvmField
    var ARM_PIVOT_SATIC_THRESH = 0.08

    val ARM_PIVOT_PID
        get() = PIDCoefficients(12.0, 0.0, 0.0)
    val ARM_EXTENSION_PID = PIDCoefficients(24.0, 0.0, 0.0)
    val SWERVE_MODULE_PID = PIDCoefficients(1.0, 0.0, 0.00)


    @JvmField
    var ARM_FINE_PIVOT_P = 6.0
    @JvmField
    var ARM_FINE_PIVOT_I = 0.0
    @JvmField
    var ARM_FINE_PIVOT_D = 0.0
    val ARM_FINE_PIVOT_PID
        get() = PIDCoefficients(ARM_FINE_PIVOT_P, ARM_FINE_PIVOT_I, ARM_FINE_PIVOT_D)

    @JvmField
    var ARM_FINE_EXTEND_P = 6.0
    @JvmField
    var ARM_FINE_EXTEND_I = 0.0
    @JvmField
    var ARM_FINE_EXTEND_D = 0.0
    val ARM_FINE_EXTEND_PID
        get() = PIDCoefficients(ARM_FINE_EXTEND_P, ARM_FINE_EXTEND_I, ARM_FINE_EXTEND_D)

    @JvmField
    val SWERVE_MAX_SLEW_RATE = 1.0

    @JvmField
    val MIN_VEL_SLEW_LIMIT = 0.25

    @JvmField
    val TELEOP_TARGET_HEADING_MAX_DIFF = 0.25

    @JvmField
    var TELEOP_HEADING_P = 6.0
    @JvmField
    var TELEOP_HEADING_I = 0.0
    @JvmField
    var TELEOP_HEADING_D = 0.0
    val TELEOP_HEADING_PID
        get() = PIDCoefficients(TELEOP_HEADING_P, TELEOP_HEADING_I, TELEOP_HEADING_D)
}