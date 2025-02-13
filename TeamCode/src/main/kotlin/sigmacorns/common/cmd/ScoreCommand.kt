package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.ds
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Tuning

enum class ScorePosition(val x: LiftPose) {
    HIGH_SPECIMEN(Tuning.specimenHighPose),
    LOW_SPECIMEN(Tuning.specimenLowPose),
    HIGH_BUCKET(Tuning.bucketHighPose),
    LOW_BUCKET(Tuning.bucketLowPose)
}

fun score(robot: Robot, dst: ScorePosition?) = (when (dst) {
    ScorePosition.HIGH_SPECIMEN, ScorePosition.LOW_SPECIMEN ->
        liftCommand(
            robot,
            (dst.x.lift + Tuning.specimentScoreOffset).cast(m)
        ) then clawCommand(robot,false)

    ScorePosition.HIGH_BUCKET, ScorePosition.LOW_BUCKET ->
        clawCommand(robot,false) then armCommand(
            robot,
            robot.arm.t.axis1.map { -it }.cast(rad),
            robot.arm.t.axis2.map { -it }.cast(rad)
        ).name("arm").timeout(7.s)
    null -> clawCommand(robot,false)
}).name("score(${dst?.name ?: "null)"})").timeout(10.s)