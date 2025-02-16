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
    var LIFT_PID: PIDCoefficients = PIDCoefficients(100.0,0.0,0.0)
    var EXTENSION_PID: PIDCoefficients = PIDCoefficients(100.0,0.0,0.0)
    var LIFT_TOLERANCE: Metre = 3.cm
    var EXTENSION_TOLERANCE: Metre = 3.cm

    var ARM_SERVO_VEL: Expression = 30.degrees/(0.2.s)
    var ARM_SERVO_TOLERANCE: Radian = 3.degrees

    var HOVER_DIST: Metre = 7.cm
    var EXTRACT_DIST = 12.cm
    var TRANSFER_POSE: LiftPose = LiftPose(2.cm,140.degrees,43.degrees)
    var TRANSFER_HOVER_POSE: LiftPose = TRANSFER_POSE
        .let { LiftPose((it.lift + HOVER_DIST).cast(m),it.arm,it.wrist) }
    var TRANSFER_EXTRACT_POSE = TRANSFER_POSE
        .let { LiftPose((it.lift + EXTRACT_DIST).cast(m),it.arm,it.wrist) }

    var INTAKE_SERVO_ACC: Expression = 100.degrees/(0.2.s)/s
    var INTAKE_SERVO_VEL: Expression = 60.degrees/(0.2.s)
    var INTAKE_SERVO_TOLERANCE: Radian = 3.degrees

    var CLAW_TIME: Second = 200.ms
    var CLAW_CLOSED: Double = 0.35
    var CLAW_OPEN: Double = 0.7

    var FLAP_TIME: Second = 300.ms
    var FLAP_CLOSED: Double = 0.6375
    var FLAP_OPEN: Double = 0.12

    var ACTIVE_POWER: Double = 1.0
    var ACTIVE_STOP_POWER: Double = -0.7
    var ACTIVE_STOP_TIME = 100.ms
    var TRANSFER_ACTIVE_TIME = 300.ms

    var specimenWallPose = LiftPose(0.cm,(-145).degrees, 40.degrees)
    var specimenHighPose = LiftPose(150.mm,(30).degrees, (30).degrees)
    var specimenLowPose = LiftPose((specimenHighPose.lift-13.inches).cast(m),(30).degrees, (30).degrees)
    var bucketHighPose: LiftPose = LiftPose(730.mm,(-45).degrees,(-25).degrees)
    var bucketLowPose: LiftPose = LiftPose((bucketHighPose.lift-44.cm).cast(m),(-45).degrees,(-25).degrees)

    var specimentScoreOffset = 220.mm

    var choreoPosPID = PIDCoefficients(20.0,0.0,1.5)
    var choreoAngPID = PIDCoefficients(15.0,0.0,0.0)

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