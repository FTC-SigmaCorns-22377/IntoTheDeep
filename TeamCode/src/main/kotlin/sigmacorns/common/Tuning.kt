package sigmacorns.common

import com.acmerobotics.dashboard.config.Config
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.degrees
import sigmacorns.common.SwerveHeading.SWERVE_HEADING_D
import sigmacorns.common.SwerveHeading.SWERVE_HEADING_I
import sigmacorns.common.SwerveHeading.SWERVE_HEADING_I_ZONE_DEG
import sigmacorns.common.SwerveHeading.SWERVE_HEADING_P
import sigmacorns.common.subsystems.arm.TrapezoidalProfile

@Config
object Tuning {
    @JvmField
    var ARM_G_MIN = -4.5

    @JvmField
    var ARM_G_MAX = -5.0

    @JvmField
    var ARM_PIVOT_PROFILE_DOWN_SPEED = 3.5
    @JvmField
    var ARM_PIVOT_PROFILE_DOWN_ACC = 4.5
    @JvmField
    var ARM_PIVOT_PROFILE_UP_SPEED = 2
    @JvmField
    var ARM_PIVOT_PROFILE_UP_ACC = 2.3

    val ARM_PIVOT_PROFILE_DOWN = TrapezoidalProfile(ARM_PIVOT_PROFILE_DOWN_SPEED.rad / s, ARM_PIVOT_PROFILE_DOWN_ACC.rad / s / s)
    val ARM_PIVOT_PROFILE_UP = TrapezoidalProfile(ARM_PIVOT_PROFILE_UP_SPEED.rad / s, ARM_PIVOT_PROFILE_UP_ACC.rad / s / s)

    @JvmField
    var ARM_FINE_DIST_DEG = 5

    val ARM_FINE_DIST
        get() = ARM_FINE_DIST_DEG.degrees

    @JvmField
    var ARM_EXTEND_FINE_DIST_CM = 5

    val ARM_FINE_EXTENSION_DIST
        get() = ARM_EXTEND_FINE_DIST_CM.cm

    @JvmField
    var ARM_PIVOT_STATIC = 0.0
    @JvmField
    var ARM_PIVOT_SATIC_THRESH = 0.08

    @JvmField
    var ARM_PIVOT_P = 12.0
    @JvmField
    var ARM_PIVOT_I = 0.0
    @JvmField
    var ARM_PIVOT_D = 0.0
    val ARM_PIVOT_PID
        get() = PIDCoefficients(ARM_PIVOT_P, ARM_PIVOT_I, ARM_PIVOT_D)

    @JvmField
    var ARM_EXTEND_P = 24.0
    @JvmField
    var ARM_EXTEND_I = 0.0
    @JvmField
    var ARM_EXTEND_D = 0.0
    val ARM_EXTENSION_PID
        get() = PIDCoefficients(ARM_EXTEND_P, ARM_EXTEND_I, ARM_EXTEND_D)

    @JvmField
    var ARM_FINE_PIVOT_P = 5.0
    @JvmField
    var ARM_FINE_PIVOT_I = 5.0
    @JvmField
    var ARM_FINE_PIVOT_D = -0.05
    val ARM_FINE_PIVOT_PID
        get() = PIDCoefficients(ARM_FINE_PIVOT_P, ARM_FINE_PIVOT_I, ARM_FINE_PIVOT_D)

    @JvmField
    var ARM_FINE_EXTEND_P = 24.0
    @JvmField
    var ARM_FINE_EXTEND_I = 1.0
    @JvmField
    var ARM_FINE_EXTEND_D = 0.0
    val ARM_FINE_EXTEND_PID
        get() = PIDCoefficients(ARM_FINE_EXTEND_P, ARM_FINE_EXTEND_I, ARM_FINE_EXTEND_D)

    val SWERVE_MODULE_PID = PIDCoefficients(1.0, 0.0, 0.00)

    @JvmField
    val SWERVE_MAX_SLEW_RATE = 1.0

    @JvmField
    val MIN_VEL_SLEW_LIMIT = 0.25

    @JvmField
    val TELEOP_TARGET_HEADING_MAX_DIFF = 0.25

    @JvmField
    var TELEOP_HEADING_P = 1.0
    @JvmField
    var TELEOP_HEADING_I = 0.0
    @JvmField
    var TELEOP_HEADING_D = 0.0
    val TELEOP_HEADING_PID
        get() = PIDCoefficients(TELEOP_HEADING_P, TELEOP_HEADING_I, TELEOP_HEADING_D)

    val SWERVE_HEADING_PID
        get() = PIDCoefficients(SWERVE_HEADING_P, SWERVE_HEADING_I, SWERVE_HEADING_D)
    val SWERVE_HEADING_I_ZONE
        get() = SWERVE_HEADING_I_ZONE_DEG.degrees
    val SWERVE_HEADING_PROFILE
        get() = TrapezoidalProfile(SwerveHeading.SPEED.rad/s,SwerveHeading.ACCELERATION.rad/s/s)
}

//TODO: only constantly update in controllers when flag is set / make nice syntax for it.

@Config
object SwerveHeading {
    @JvmField
    var SWERVE_HEADING_P = 10.0
    @JvmField
    var SWERVE_HEADING_I = 2.0
    @JvmField
    var SWERVE_HEADING_D = 0.0
    @JvmField
    var SWERVE_HEADING_I_ZONE_DEG = 5.0

    @JvmField
    var SPEED = 1.0
    @JvmField
    var ACCELERATION = 1.0
}