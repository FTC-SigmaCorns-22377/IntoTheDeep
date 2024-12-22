package sigmacorns.common.subsystems.swerve

import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math.Vector2

typealias PositionState = Pose
typealias PositionInput = SwerveTarget
typealias PositionTarget = Pose

class PositionController() : Controller<PositionState, PositionInput, PositionTarget>() {
    override var target: PositionTarget = Pose(0, 0 , 0)
    override var output: PositionInput = SwerveTarget(Vector2(0, 0), 0.0, false)
    override var position: PositionState = Pose(0, 0, 0)

    var positionPIDX: PIDController = PIDController(0, 0, 0)
    var positionPIDY: PIDController = PIDController(0, 0, 0)
    var positionPIDAngle: PIDController = PIDController(0, 0, 0)

    override fun copy(): Controller<PositionState, PositionInput, PositionTarget> {
        TODO("Not yet implemented")
    }

    override fun update(deltaTime: Double): PositionInput {
        val vX = positionPIDX.updateStateless(deltaTime, position.x, target.x)
        val vY = positionPIDY.updateStateless(deltaTime, position.y, target.y)
        val vAngle = positionPIDAngle.updateStateless(deltaTime, position.angle, target.angle)
        return SwerveTarget(Vector2(vX, vY), vAngle, lockWheels = false)
    }

}