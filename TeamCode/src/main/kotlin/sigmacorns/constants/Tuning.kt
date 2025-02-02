package sigmacorns.constants

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.derived.Radian
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose

object Tuning {
    //-50, 50
//    var LIFT_PID: PIDCoefficients = PIDCoefficients(-50.0,0.0,0.0)
//    var EXTENSION_PID: PIDCoefficients = PIDCoefficients(50.0,0.0,2.0)

    var LIFT_PID: PIDCoefficients = PIDCoefficients(100.0,0.0,0.0)
    var EXTENSION_PID: PIDCoefficients = PIDCoefficients(100.0,0.0,0.0)
    var LIFT_TOLERANCE: Metre = 3.cm
    var EXTENSION_TOLERANCE: Metre = 3.cm

    var ARM_SERVO_ACC: Expression = 50.degrees/(0.2.s)/s
    var ARM_SERVO_VEL: Expression = 30.degrees/(0.2.s)
    var ARM_SERVO_TOLERANCE: Radian = 3.degrees

    var TRANSFER_EXTEND = (-10).mm
    var TRANSFER_DIST = 3.cm
    var TRANSFER_ANGLE_INTAKE = 0.degrees
    var TRANSFER_CLAW_ANGLE = 180.degrees

    var TRANSFER_LIFT: Metre
    var TRANSFER_ARM: DiffyOutputPose
    var HOVER_DIST: Metre = 7.cm
    var HOVER_LIFT: Metre
    var EXTRACT_DIST = 15.cm
    var EXTRACT_LIFT: Metre

    init {
//        val samplePos = Physical.INTAKE_END + IntakeAngleKinematics.offsetFromSlideEnd(TRANSFER_ANGLE_INTAKE, TRANSFER_DIST)
//        val liftPose = LiftKinematics.inverse(LiftScoringTarget(samplePos, TRANSFER_CLAW_ANGLE))
        val transferLiftPose = LiftPose(2.cm,140.degrees,43.degrees)

        TRANSFER_LIFT = transferLiftPose.lift
        TRANSFER_ARM = DiffyOutputPose(transferLiftPose.arm, transferLiftPose.wrist)
        HOVER_LIFT = (TRANSFER_LIFT + HOVER_DIST).cast(m)
        EXTRACT_LIFT = (TRANSFER_LIFT + EXTRACT_DIST).cast(m)
    }

    var INTAKE_SERVO_ACC: Expression = 100.degrees/(0.2.s)/s
    var INTAKE_SERVO_VEL: Expression = 60.degrees/(0.2.s)
    var INTAKE_SERVO_TOLERANCE: Radian = 3.degrees

    var CLAW_CLOSED: Double = 0.35
    var CLAW_OPEN: Double = 0.7

    var ACTIVE_POWER: Double = -0.7


    var INTAKE_OVER_POS = 0.25 to 0.57
    var INTAKE_INTER_POS = 0.1015 to 0.1855
    var INTAKE_BACK_POS = 0.0 to 0.805

    var specimenWallPose = LiftPose(0.cm,(-145).degrees, 55.degrees)
    var specimenHighPose = LiftPose(170.mm,(30).degrees,(65).degrees)
    var specimenLowPose = LiftPose(170.mm,(30).degrees,(65).degrees)

    var specimentScoreOffset = 170.mm

    var bucketHighPose: LiftPose = LiftPose(550.mm,(-45).degrees,(-45).degrees)
    var bucketLowPose: LiftPose = LiftPose(400.mm,(-45).degrees,(-45).degrees)
}