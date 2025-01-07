package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math.Angle
import net.unnamedrobotics.lib.math.radians
import kotlin.math.max
import kotlin.math.min

data class ClawPose(val pitch: Radian, val roll: Radian)
data class ServoTarget(val lAngle: Double, val rAngle: Double)

typealias ClawInput = ServoTarget
typealias ClawTarget = ClawPose

class ClawController() : Controller<Unit, ClawInput, ClawTarget>() {
    override var target: ClawTarget = ClawPose(0.rad, 0.rad)
    override var output: ClawInput = ServoTarget(0.0, 0.0)
    override var position: Unit = Unit

    override fun copy(): Controller<Unit, ClawInput, ClawTarget> {
        TODO("Not yet implemented")
    }

    override fun update(deltaTime: Double): ClawInput {
        var desiredPitch = target.roll.value
        var desiredWrist = target.pitch.value
        desiredPitch = max(-90.0, min(90.0, desiredPitch))
        desiredWrist = max(-90.0, min(90.0, desiredWrist))

        // Convert desired angles back to normalized servo positions
        val averagePosition = (desiredPitch + 90.0) / 180.0 // Normalized average for pitch
        val diffPosition = desiredWrist / 180.0 // Normalized difference for wrist

        // Calculate individual servo positions
        val positionL = averagePosition - (diffPosition / 2.0) // Left servo position
        val positionR = averagePosition + (diffPosition / 2.0) // Right servo position

        // Convert to 0-355 range and set servo positions
        return ServoTarget(positionL, positionR)
    }

}