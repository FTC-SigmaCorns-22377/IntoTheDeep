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
import sigmacorns.common.kinematics.LiftPose

object Tuning {
    //-50, 50
//    var LIFT_PID: PIDCoefficients = PIDCoefficients(-50.0,0.0,0.0)
//    var EXTENSION_PID: PIDCoefficients = PIDCoefficients(50.0,0.0,2.0)

    var LIFT_PID: PIDCoefficients = PIDCoefficients(100.0,0.0,0.0)
    var EXTENSION_PID: PIDCoefficients = PIDCoefficients(100.0,0.0,0.0)
    var LIFT_TOLERANCE: Metre = 3.cm
    var EXTENSION_TOLERANCE: Metre = 3.cm

    var ARM_SERVO_VEL: Expression = 30.degrees/(0.2.s)
    var ARM_SERVO_TOLERANCE: Radian = 3.degrees

    var HOVER_DIST: Metre = 10.cm
    var EXTRACT_DIST = 15.cm
    var TRANSFER_POSE: LiftPose = LiftPose(2.cm,140.degrees,43.degrees)
    var TRANSFER_HOVER_POSE: LiftPose = TRANSFER_POSE
        .let { LiftPose((it.lift + HOVER_DIST).cast(m),it.arm,it.wrist) }
    var TRANSFER_EXTRACT_POSE = TRANSFER_POSE
        .let { LiftPose((it.lift + EXTRACT_DIST).cast(m),it.arm,it.wrist) }

    var INTAKE_SERVO_ACC: Expression = 100.degrees/(0.2.s)/s
    var INTAKE_SERVO_VEL: Expression = 60.degrees/(0.2.s)
    var INTAKE_SERVO_TOLERANCE: Radian = 3.degrees

    var CLAW_TIME: Second = 300.ms
    var CLAW_CLOSED: Double = 0.35
    var CLAW_OPEN: Double = 0.7

    var ACTIVE_POWER: Double = -1.0
    var ACTIVE_STOP_POWER: Double = 0.4
    var ACTIVE_STOP_TIME = 300.ms
    var TRANSFER_ACTIVE_TIME = 300.ms

    enum class IntakePosition(val x: Radian) {
        OVER(0.rad),
        BACK((-0.43).rad),
        ACTIVE((-0.43).rad)
    }

    var specimenWallPose = LiftPose(0.cm,(-145).degrees, 55.degrees)
    var specimenHighPose = LiftPose(110.mm,(30).degrees, (30).degrees)
    var specimenLowPose = LiftPose(170.mm,(30).degrees, (30).degrees)
    var bucketHighPose: LiftPose = LiftPose(750.mm,(-45).degrees,(-25).degrees)
    var bucketLowPose: LiftPose = LiftPose(400.mm,(-45).degrees,(-25).degrees)

    var postTransferPose = LiftPose(TRANSFER_EXTRACT_POSE.lift, bucketLowPose.arm, bucketLowPose.wrist)
    var asdf = LiftPose(150.mm,(30).degrees, (50).degrees)

    enum class TiltPositions(val x: Double) {
        STRAIGHT(0.65),
        UP(0.0),
        DOWN(1.0)
    }

    var specimentScoreOffset = 200.mm


}