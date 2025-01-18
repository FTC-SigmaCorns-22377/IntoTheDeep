package sigmacorns.common

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import net.unnamedrobotics.lib.math2.inches
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.arm.ScoringTarget

object ScoringPresets {
    val SPECIMEN_LOW = 13.inches
    val SPECIMEN_HIGH = 26.inches

    val BUCKET_LOW = 25.75.inches
    val BUCKET_HIGH = 43.inches

    val BUCKET_OFFSET = 5.inches

    val SAMPLE_LOW = BUCKET_LOW + BUCKET_OFFSET
    val SAMPLE_HIGH = BUCKET_HIGH + BUCKET_OFFSET

    val SAMPLE_PLACE_DISTANCE = (-7).inches
    val SAMPLE_PITCH = 30.degrees
    val SAMPLE_ROLL = 90.degrees

    val SPECIMEN_PLACE_DISTANCE = (10).inches
    val SPECIMEN_PITCH = 0.rad

    val SUBMERSIBLE_TRANSITION_HEIGHT = 6.inches

    val HOVER_HEIGHT = 2.inches
    val HOVER_PITCH = 175.degrees

    val GRAB_HEIGHT = 1.inches

    fun placeHeight(height: Expression, distance: Expression, pitch: Radian, roll: Radian = 0.rad): ScoringPose
        = ScoringKinematics.inverse(ScoringTarget(
            vec2(0.m,0.m),
            0.rad,
            vec3(distance.cast(m), 0.m,height.cast(m)),
            pitch,
            roll
        ))

    fun placeHighSample() = placeHeight(SAMPLE_HIGH, SAMPLE_PLACE_DISTANCE, SAMPLE_PITCH, SAMPLE_ROLL)
    fun placeLowSample() = placeHeight(SAMPLE_LOW, SAMPLE_PLACE_DISTANCE, SAMPLE_PITCH, SAMPLE_ROLL)
    fun placeHighSpecimen() = placeHeight(SPECIMEN_HIGH, SPECIMEN_PLACE_DISTANCE, SPECIMEN_PITCH)
    fun placeLowSpecimen() = placeHeight(SPECIMEN_LOW, SPECIMEN_PLACE_DISTANCE, SPECIMEN_PITCH)

    fun placeOverSample(distance: Metre) = placeHeight(HOVER_HEIGHT, distance, HOVER_PITCH)
    fun placeOverSubmersible(distance: Metre) = placeHeight(SUBMERSIBLE_TRANSITION_HEIGHT,distance, HOVER_PITCH)
    fun grabSamplePose(distance: Metre) = placeHeight(GRAB_HEIGHT,distance, HOVER_PITCH)
}

object AutoSamples {
    val SAMPLE_Y_1 = 60.cm -1.inches
    val SAMPLE_X = 120.cm - 2.inches
    val SAMPLE_HEIGHT = 1.5.inches

    val BLUE_BUCKET_SAMPLE_1 = vec3(24.inches*3 - SAMPLE_X, SAMPLE_Y_1, SAMPLE_HEIGHT/2.0)
    val BLUE_BUCKET_SAMPLE_2 = vec3(BLUE_BUCKET_SAMPLE_1.x, BLUE_BUCKET_SAMPLE_1.y + 10.inches, BLUE_BUCKET_SAMPLE_1.z)
    val BLUE_BUCKET_SAMPLE_3 = vec3(BLUE_BUCKET_SAMPLE_2.x, BLUE_BUCKET_SAMPLE_2.y + 10.inches, BLUE_BUCKET_SAMPLE_2.z)
}