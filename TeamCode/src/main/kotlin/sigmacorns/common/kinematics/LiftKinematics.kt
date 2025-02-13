package sigmacorns.common.kinematics

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.spherical
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z
import net.unnamedrobotics.lib.physics.Kinematics
import sigmacorns.constants.Physical
import kotlin.math.absoluteValue

data class LiftScoringTarget(val pos: Vector3, val phi: Radian)
data class LiftPose(val lift: Metre, val arm: Radian, val wrist: Radian)

object LiftKinematics: Kinematics<LiftPose, LiftScoringTarget> {
    override fun forward(x: LiftPose): LiftScoringTarget {
        var end = Physical.ARM_AXLE_POS
        end += spherical(x.lift,0.rad,Physical.LIFT_ANGLE)
        end += spherical(Physical.ARM_LENGTH,0.rad,x.arm)
        end += spherical(Physical.CLAW_LENGTH,0.rad,x.wrist)

        return LiftScoringTarget(end, x.wrist)
    }

    override fun inverse(x: LiftScoringTarget): LiftPose {
        val armEnd = x.pos - spherical(Physical.CLAW_LENGTH,0.rad,x.phi)
        var v = armEnd - Physical.ARM_AXLE_POS
        val offset = (Physical.ARM_LENGTH.pow(2) - v.x.pow(2)).pow(0.5)

        // two solutions here, we want to take the one with a lower lift pos unless it would be under min.
        var liftHeight = v.z - offset
        if(liftHeight < 0.m) liftHeight = v.z + offset

        v = vec3(v.x,v.y,liftHeight)

        var arm = v.phi()
        if(v.theta().map { it.absoluteValue } > 90.degrees) arm = arm.map { -it }

        val wrist = (x.phi - arm).normalizeRadian()

        return LiftPose(liftHeight.cast(m),arm,wrist)
    }
}