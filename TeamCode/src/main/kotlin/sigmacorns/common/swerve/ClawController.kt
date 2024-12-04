package sigmacorns.common.swerve

import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math.Angle
import net.unnamedrobotics.lib.math.radians

data class ClawPose(val pitch: Angle, val roll: Angle)
data class ServoTarget(val lAngle: Angle, val rAngle: Angle)

typealias ClawInput = ServoTarget
typealias ClawTarget = ClawPose

class ClawController() : Controller<Unit, ClawInput, ClawTarget>() {
    override var target: ClawTarget = ClawPose(0.radians, 0.radians)
    override var output: ClawInput = ServoTarget(0.radians, 0.radians)
    override var position: Unit = Unit

    override fun copy(): Controller<Unit, ClawInput, ClawTarget> {
        TODO("Not yet implemented")
    }

    override fun update(deltaTime: Double): ClawInput {
        val lAngle = target.roll + target.pitch
        val rAngle = target.roll - target.pitch
        return ServoTarget(lAngle, rAngle)
    }

}