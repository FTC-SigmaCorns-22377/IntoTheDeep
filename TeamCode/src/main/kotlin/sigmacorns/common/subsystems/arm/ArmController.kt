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
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.lerp
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.spherical
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.archetypes.Arrows3D
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.archetypes.FillMode
import org.joml.Quaterniond
import sigmacorns.common.Constants
import sigmacorns.common.Constants.CLAW_CLOSED
import sigmacorns.common.Constants.CLAW_OPEN
import sigmacorns.common.Tuning
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin

typealias ArmMotorPowers = List<Volt>

data class ArmState(val motorPos: DiffyInputPose)
data class ArmInput(val motors: ArmMotorPowers, val servoTarget: List<Number>, val clawTarget: Double)
data class ArmTarget(var pivot: Radian, var extension: Metre, var pitch: Radian, var roll: Radian, var isOpen: Boolean)

val boxTubeKinematics = DiffyKinematics(Constants.ARM_PIVOT_RATIO,Constants.ARM_EXTENSION_RATIO)
val clawKinematics = DiffyKinematics(Constants.CLAW_PITCH_RATIO,Constants.CLAW_ROLL_RATIO)

class ArmController()
    : Controller<ArmState,ArmInput,ArmTarget>(), Rerunable {
    override lateinit var position: ArmState

    override var target: ArmTarget = ArmTarget(0.rad,400.mm,0.rad,0.rad,false)

    var armDiffyController = PIDDiffyController(boxTubeKinematics,Tuning.ARM_PIVOT_PID,Tuning.ARM_EXTENSION_PID)

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
        val useProfile = (pos.axis1.value - pivot.value).absoluteValue > Tuning.ARM_FINE_DIST.value

        if(!useProfile) profile = null
        if(useProfile && (profile==null  || ((profile!!)(1000.s) - target.pivot).value.absoluteValue > Tuning.ARM_FINE_DIST.value.absoluteValue)) {
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

        scalar("rawPivotPower", armDiffyController.lastRawPivotPower.value)
        scalar("rawExtensionPower", armDiffyController.lastRawExtensionPower.value)

        scalar("boundedPivotPower", armDiffyController.lastBoundedPivotPower.value)
        scalar("boundedExtensionPower", armDiffyController.lastBoundedExtensionPower.value)
    } }
}

// TODO: go back to using fixed lib versions of these functions :)
fun <T> ClosedRange<T>.inverseLerp(res: Number) where T: Comparable<T>, T: Number
        = (res.toDouble()-start.toDouble())/(endInclusive.toDouble()-start.toDouble())


fun <T> mapRanges(from: ClosedRange<T>, to: ClosedRange<T>): (Number)->Number
        where T : Comparable<T>, T: Number = {
    to.lerp(from.inverseLerp(it.toDouble()))
}