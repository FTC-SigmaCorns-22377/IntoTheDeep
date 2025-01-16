package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.pow
import eu.sirotin.kotunil.core.unaryMinus
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math2.Dim2
import net.unnamedrobotics.lib.math2.Dim3
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Vector
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.rotateZ
import net.unnamedrobotics.lib.math2.spherical
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.math2.xy
import net.unnamedrobotics.lib.math2.z
import net.unnamedrobotics.lib.physics.Kinematics
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import sigmacorns.common.Constants
import kotlin.math.absoluteValue
import kotlin.math.asin

data class ScoringTarget(
    val robotPos: Vector<Dim2>,
    val robotHeading: Radian,
    val samplePos: Vector<Dim3>,
    val pitch: Radian,
    val roll: Radian
)

data class ScoringPose(
    val robotPos: Vector<Dim2>,
    val theta: Radian,
    val extension: Metre,
    val pivot: Radian,
    val roll: Radian,
    val pitch: Radian
) {
    fun armTarget(openClaw: Boolean) = ArmTarget(pivot,extension,pitch,roll,openClaw)
    val poseTarget: Transform2D
        get() = Transform2D(robotPos,theta)
}

object ScoringKinematics: Kinematics<ScoringPose, ScoringTarget> {
    override fun forward(x: ScoringPose): ScoringTarget {
        val offsetPos = (x.robotPos.withZ(0.m) +
                Constants.AXLE_CENTER.rotateZ(x.theta)) +
                spherical(Constants.ARM_OFFSET,x.theta,(x.pivot+90.degrees).cast(rad))
        val armEndPos = offsetPos +
                spherical(x.extension, x.theta, x.pivot)
        val clawEndPos = armEndPos +
                spherical(Constants.CLAW_LENGTH,x.theta,(x.pivot+x.pitch).cast(rad))
        TODO()
    }

    lateinit var lastAxleRelative: Vector3
    lateinit var lastOffsetPos: Vector3
    lateinit var shouldBe: Vector3

    override fun inverse(x: ScoringTarget): ScoringPose {
        val robotRelSamplePos = x.samplePos - x.robotPos.withZ(0.m)

        var theta = robotRelSamplePos.theta()
        val flipped = normalizeRadians(x.robotHeading.value-theta.value).rad.map { it.absoluteValue } > 90.degrees
        if(flipped) theta = (theta + 180.degrees).cast(rad)

        val axleRelativeSamplePos = x.samplePos - (x.robotPos.withZ(0.m) + Constants.AXLE_CENTER.rotateZ(theta))
        lastAxleRelative = axleRelativeSamplePos
        val armEndPos = axleRelativeSamplePos + spherical(-Constants.CLAW_LENGTH*0,theta,x.pitch)

        val a = (Constants.ARM_OFFSET/axleRelativeSamplePos.magnitude()).map { asin(it) }.cast(rad)
        val pivot = armEndPos.phi().plus(if(flipped) a else -a).cast(rad)

        val extension = (axleRelativeSamplePos.sqrMagnitude().minus(Constants.ARM_OFFSET.pow(2))).pow(0.5).cast(m)

        return ScoringPose(
            x.robotPos,
            theta,
            extension,
            pivot,
            x.roll,
            (x.pitch - pivot).cast(rad)
        )
    }
}
