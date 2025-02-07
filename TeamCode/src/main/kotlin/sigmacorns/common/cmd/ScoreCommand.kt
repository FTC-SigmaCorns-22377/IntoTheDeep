package sigmacorns.common.cmd

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Tuning

enum class ScorePositions(val x: LiftPose) {
    HIGH_SPECIMEN(Tuning.specimenHighPose),
    LOW_SPECIMEN(Tuning.specimenLowPose),
    HIGH_BUCKET(Tuning.bucketHighPose),
    LOW_BUCKET(Tuning.bucketLowPose)
}

fun score(robot: Robot, dst: ScorePositions?) = when (dst) {
    ScorePositions.HIGH_SPECIMEN, ScorePositions.LOW_SPECIMEN ->
        liftCommand(
            robot,
            (dst.x.lift + Tuning.specimentScoreOffset).cast(m)
        ) then clawCommand(robot,false)

    ScorePositions.HIGH_BUCKET, ScorePositions.LOW_BUCKET ->
        clawCommand(robot,false) then wait(300.ms) then armCommand(
            robot,
            robot.arm.t.axis1.map { -it }.cast(rad),
            robot.arm.t.axis2.map { -it }.cast(rad)
        )
    null -> clawCommand(robot,false)
}