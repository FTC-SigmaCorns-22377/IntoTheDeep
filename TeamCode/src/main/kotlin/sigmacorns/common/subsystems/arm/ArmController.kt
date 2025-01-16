package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.control.controller.transfer
import net.unnamedrobotics.lib.math.RGBA
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.cos
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.lerp
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.sin
import net.unnamedrobotics.lib.math2.spherical
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.archetypes.Arrows3D
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.archetypes.FillMode
import org.joml.AxisAngle4d
import org.joml.Quaterniond
import sigmacorns.common.Constants
import sigmacorns.common.Constants.CLAW_CLOSED
import sigmacorns.common.Constants.CLAW_OPEN
import sigmacorns.common.Tuning
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin

typealias ArmMotorPowers = List<Volt>

data class ArmState(val motorPos: DiffyInputPose)
data class ArmInput(val motors: ArmMotorPowers, val servoTarget: List<Number>, val clawTarget: Double)
data class ArmTarget(var pivot: Radian, var extension: Metre, var pitch: Radian, var roll: Radian, val isOpen: Boolean)

val boxTubeKinematics = DiffyKinematics(Constants.ARM_PIVOT_RATIO,Constants.ARM_EXTENSION_RATIO)
val clawKinematics = DiffyKinematics(Constants.CLAW_PITCH_RATIO,Constants.CLAW_ROLL_RATIO)

class ArmController()
    : Controller<ArmState,ArmInput,ArmTarget>(), Rerunable {
    override lateinit var position: ArmState

    override var target: ArmTarget = ArmTarget(0.rad,400.mm,0.rad,0.rad,false)

    var armDiffyController = pidDiffyController(boxTubeKinematics,Tuning.ARM_PIVOT_PID,Tuning.ARM_EXTENSION_PID)

    override var output: ArmInput = ArmInput(List(4) { 0.V }, List(2) { 0.0 }, Constants.CLAW_OPEN)

    override fun copy() = ArmController()

    val mapServo1 = mapRanges(Constants.CLAW_SERVO_1_BOUNDS.let { it.min.value..it.max.value },0.0..1.0)
    val mapServo2 = mapRanges(Constants.CLAW_SERVO_2_BOUNDS.let { it.min.value..it.max.value },0.0..1.0)

    var profileT = 0.s
    var profile: ((Second) -> Expression)? = null

    override fun update(deltaTime: Double): ArmInput {
        var pivot = Constants.ARM_PIVOT_BOUNDS.apply(target.pivot)
        var extension = Constants.ARM_EXTENSION_BOUNDS.apply(target.extension)

        val pos = boxTubeKinematics.forward(position.motorPos)
        val useProfile = (pos.axis1.value - pivot.value).absoluteValue > Tuning.ARM_PROFILE_DIST.value

        if(!useProfile) profile = null
        if(useProfile && (profile==null  || ((profile!!)(1000.s) - target.pivot).value.absoluteValue > Tuning.ARM_PROFILE_DIST.value.absoluteValue)) {
            profile =
                (
                    if(pivot.map { it.absoluteValue } > pos.axis1.map { it.absoluteValue })
                        Tuning.ARM_PIVOT_PROFILE_DOWN
                    else
                        Tuning.ARM_PIVOT_PROFILE_UP
                ).new(pos.axis1,target.pivot)
            profileT = 0.s
        }

        if(useProfile) {
            pivot = (profile!!)(profileT).cast(rad)
            profileT = (profileT + deltaTime.s).cast(s)
        }

        val motorPowers = armDiffyController.updateStateless(deltaTime,position.motorPos,DiffyOutputPose(pivot,extension))

        val servoPos = clawKinematics.inverse(DiffyOutputPose(
            Constants.CLAW_PITCH_BOUNDS.apply(target.pitch),
            Constants.CLAW_ROLL_BOUNDS.apply(target.roll)
        ))

        val servo1 = (mapServo1)(servoPos.axis1.value)
        val servo2 = (mapServo2)(servoPos.axis2.value)

        val clawServo = if (target.isOpen) CLAW_OPEN else CLAW_CLOSED

        return ArmInput(motorPowers, listOf(servo1,servo2), clawServo)
    }

    context(RerunPrefix, RerunConnection) override fun log(name: String) { prefix(name) {
        log("axle") {
            Boxes3D(
                halfSizes = listOf(vec3(
                    x = Constants.AXLE_VISUAL_RADIUS,
                    y = Constants.DRIVEBASE_SIZE.y/2,
                    z = Constants.AXLE_VISUAL_RADIUS
                )),
                centers = listOf(Constants.AXLE_CENTER),
                fillMode = FillMode.MajorWireframe
            )
        }

        val logBoxtube = { name: String, pivot: Radian, extension: Metre, color: RGBA, fillMode: FillMode -> log(name) {
            val extensions = listOf(0,1.0/2.0,1).map {
                val extensionPastTopOfFirst = extension - (Constants.BOXTUBE_SECTION_LENGTH - Constants.BOXTUBE_1_VISUAL_OFFSET)
                - Constants.BOXTUBE_1_VISUAL_OFFSET + it*extensionPastTopOfFirst }

            val centers = extensions.map {
                Constants.AXLE_CENTER +
                        spherical(Constants.ARM_OFFSET,0.rad,(pivot+90.degrees).cast(rad)) +
                        spherical(it, 0.rad,pivot)
            }

            val sizes = Constants.BOXTUBE_VISUAL_SIZES.map { vec3(it/2,it/2, Constants.BOXTUBE_SECTION_LENGTH) }

            val rotations = List(3) {
                Quaterniond().setAngleAxis(pivot.value,0.0,1.0,0.0)
            }

            Boxes3D(
                centers = centers,
                halfSizes = sizes,
                rotations = rotations,
                colors = List(3) { color },
                fillMode = FillMode.MajorWireframe
            )
        } }

        val curPos = boxTubeKinematics.forward(position.motorPos)

        logBoxtube("position",curPos.axis1.cast(rad),curPos.axis2.cast(m), RGBA(0.7,0.7,0.7,1.0), FillMode.MajorWireframe)
        logBoxtube("target",target.pivot,target.extension, RGBA(0.0,255.0,0.0,255.0), FillMode.MajorWireframe)

        if (profile != null ) {
            val phi = (profile!!)(profileT)
            log("profiledTarget") {
                Arrows3D(
                    vecs = listOf(spherical(curPos.axis2, 0.rad, phi.cast(rad))),
                    origins = listOf(Constants.AXLE_CENTER + spherical(Constants.ARM_OFFSET,0.rad, (phi + 90.degrees).cast(rad)))
                )
            }

            scalar("targetProfiled",phi.value)
        }

        scalar("rawPivotPower", lastRawPivotPower.value)
        scalar("rawExtensionPower", lastRawExtensionPower.value)

        scalar("boundedPivotPower", lastBoundedPivotPower.value)
        scalar("boundedExtensionPower", lastBoundedExtensionPower.value)
    } }

    private var lastRawPivotPower = 0.0.V
    private var lastRawExtensionPower = 0.0.V
    private var lastBoundedPivotPower = 0.0.V
    private var lastBoundedExtensionPower = 0.0.V

    fun pidDiffyController(kinematics: DiffyKinematics, axis1PIDCoefficients: PIDCoefficients, axis2PIDCoefficients: PIDCoefficients)
            : Controller<DiffyInputPose,ArmMotorPowers,DiffyOutputPose> {
        val pid1 = PIDController(axis1PIDCoefficients)
        val pid2 = PIDController(axis2PIDCoefficients)

        return transfer { dt: Double, x: DiffyInputPose, t: DiffyOutputPose ->
            pid1.coefficients = Tuning.ARM_PIVOT_PID
            pid2.coefficients = Tuning.ARM_EXTENSION_PID

            val tBounded = DiffyOutputPose(
                Constants.ARM_PIVOT_BOUNDS.apply(t.axis1.cast(rad)),
                Constants.ARM_EXTENSION_BOUNDS.apply(t.axis2.cast(m))
            )

            val powers = kinematics.forward(x).let {
                var pivot = pid1.updateStateless(dt,it.axis1.value,tBounded.axis1.value).V
                var extension = pid2.updateStateless(dt,it.axis2.value,tBounded.axis2.value).V

                if((it.axis1.value-target.pivot.value).absoluteValue < Tuning.ARM_PIVOT_SATIC_THRESH)
                    pivot = pivot.map { it + it.sign*Tuning.ARM_PIVOT_STATIC }

                lastRawPivotPower = pivot
                lastRawExtensionPower = extension

                val pivotOverMax = it.axis1 > Constants.ARM_PIVOT_BOUNDS.max - Constants.ARM_SAFE_PIVOT_THRESH
                val pivotUnderMin = it.axis1 < Constants.ARM_PIVOT_BOUNDS.min + Constants.ARM_SAFE_PIVOT_THRESH
                val extensionOverMax = it.axis2 > Constants.ARM_EXTENSION_BOUNDS.max - Constants.ARM_SAFE_EXTENSION_THRESH
                val extensionUnderMin = it.axis2 < Constants.ARM_EXTENSION_BOUNDS.min + Constants.ARM_SAFE_EXTENSION_THRESH

                val pivotPowerBounds = Bounds(
                    if(pivotUnderMin) Constants.ARM_SAFE_PIVOT_POWER.map { v -> -v } else (-Double.MAX_VALUE).V,
                    if(pivotOverMax) Constants.ARM_SAFE_PIVOT_POWER else Double.MAX_VALUE.V
                )

                val extensionPowerBounds = Bounds(
                    if(extensionUnderMin) Constants.ARM_SAFE_EXTENSION_POWER.map { v -> -v } else (-Double.MAX_VALUE).V,
                    if(extensionOverMax) Constants.ARM_SAFE_EXTENSION_POWER else Double.MAX_VALUE.V
                )

                val t = Constants.ARM_EXTENSION_BOUNDS.let { b ->
                    (it.axis2.value-b.min.value)/(b.max.value-b.min.value)
                }

                val gE = (Tuning.ARM_G_MIN..Tuning.ARM_G_MAX).lerp(t.clampMagnitude(1.0))

                var g = sin(it.axis1.value)*gE.V*it.axis2.value

//                if(it.axis1.value.absoluteValue < target.pivot.value.absoluteValue) g = 0.V
                pivot = pivotPowerBounds.apply((pivot + g).cast(V))
                extension = extensionPowerBounds.apply(extension)


                lastBoundedPivotPower = pivot
                lastBoundedExtensionPower = extension

                val powers = listOf(pivot+extension,pivot-extension)

                val scalar = min(1.0,Constants.ARM_MAX_MOTOR_POWER.value/powers.maxOfOrNull { abs(it.value) }!! )
                powers.map { p -> (p*scalar).cast(V) }
            }

            powers
        }
    }
}

fun <T> ClosedRange<T>.inverseLerp(res: Number) where T: Comparable<T>, T: Number
        = (res.toDouble()-start.toDouble())/(endInclusive.toDouble()-start.toDouble())


fun <T> mapRanges(from: ClosedRange<T>, to: ClosedRange<T>): (Number)->Number
        where T : Comparable<T>, T: Number = {
    to.lerp(from.inverseLerp(it.toDouble()))
}