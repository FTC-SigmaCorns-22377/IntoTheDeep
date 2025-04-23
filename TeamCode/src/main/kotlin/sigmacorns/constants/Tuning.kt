package sigmacorns.constants

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inches
import sigmacorns.common.kinematics.LiftPose
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

object Tuning {
    var LIFT_PID: PIDCoefficients = PIDCoefficients(150.0,0.0,0.0)
    var EXTENSION_PID: PIDCoefficients = PIDCoefficients(150.0,0.0,0.0)
    var LIFT_TOLERANCE: Metre = 3.cm
    var EXTENSION_TOLERANCE: Metre = 3.cm
    var SLIDE_RESET_POWER = -0.5

    var ARM_SERVO_VEL: Expression = 30.degrees/(0.2.s)
    var ARM_SERVO_TOLERANCE: Radian = 3.degrees

    var HOVER_DIST: Metre = 5.cm
    var EXTRACT_DIST = 9.cm
    var TRANSFER_POSE: LiftPose = LiftPose(0.cm,2.333.rad,(3.36.rad - 6.0.degrees).cast(rad))
    var TRANSFER_HOVER_POSE: LiftPose = TRANSFER_POSE
        .let { LiftPose((it.lift + HOVER_DIST).cast(m),it.arm,it.wrist) }
    var TRANSFER_EXTRACT_POSE = TRANSFER_POSE
        .let { LiftPose((it.lift + EXTRACT_DIST).cast(m),it.arm,it.wrist) }

    var PTO_L_ACTIVE = 0.69
    var PTO_L_INACTIVE = 0.9

    var PTO_R_ACTIVE = 0.64
    var PTO_R_INACTIVE = 0.47

    var CLAW_TIME: Second = 400.ms
    var CLAW_CLOSED: Double = 0.69
    var CLAW_OPEN: Double = 1.0

    var FLAP_TIME: Second = 600.ms
    var FLAP_CLOSED: Double = 0.66
    var FLAP_EJECT: Double = 0.5
    var FLAP_OPEN: Double = 0.0

    var ACTIVE_POWER: Double = 0.7
    var ACTIVE_STOP_POWER: Double = -0.7
    var ACTIVE_STOP_TIME = 100.ms
    var TRANSFER_ACTIVE_TIME = 300.ms

    var specimenWallPose = LiftPose(0.cm,(-145).degrees, (-113).degrees)
    var specimenHighPose = LiftPose(150.mm,(30).degrees, (60).degrees)
    var specimenLowPose = LiftPose((specimenHighPose.lift-13.inches).cast(m),(30).degrees, (60).degrees)
    var bucketHighPose: LiftPose = LiftPose(670.mm,(-45).degrees,(-65).degrees)
    var bucketLowPose: LiftPose = LiftPose((bucketHighPose.lift-44.cm).cast(m),(-45).degrees,(-65).degrees)

    var specimentScoreOffset = 200.mm

    var choreoPosPID = PIDCoefficients(12.0,0.0,2.5)
    var choreoAngPID = PIDCoefficients(15.0,0.0,1.0)
    var choreoKA = 0.5
    var choreoKV = 0.7

    var choreoPosThresh = 3.cm
    var choreoAngThresh = 5.degrees
    var choreoVelThresh = 10.cm/s
    var choreoAngVelThresh = 10.degrees/s

    var STICK_DEADZONE = 0.1
    var STICK_EXP = 2.0

    var stickProfile: (Float) -> Double = {
        val deadZoneAdjustedPow = max(0.0,(abs(it) - STICK_DEADZONE)/(1.0- STICK_DEADZONE))
        sign(it).toDouble()*deadZoneAdjustedPow.pow(STICK_EXP)
    }
}