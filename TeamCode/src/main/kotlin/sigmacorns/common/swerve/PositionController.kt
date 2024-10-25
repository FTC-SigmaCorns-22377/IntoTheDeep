package sigmacorns.common.swerve

import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math.Vector2

typealias PointState = Pose
typealias PointInput = SwerveTarget
typealias PointTarget = Pose

class PointController() : Controller<PointState, PointInput, PointTarget>() {
    override var target: PointTarget = Pose(0, 0 , 0)
    override var output: PointInput = SwerveTarget(Vector2(0, 0), 0.0, false)
    override var position: PointState = Pose(0, 0, 0)

    var pointPIDX: PIDController = PIDController(0, 0, 0)
    var pointPIDY: PIDController = PIDController(0, 0, 0)
    var pointPIDTheta: PIDController = PIDController(0, 0, 0)



    override fun copy(): Controller<PointState, PointInput, PointTarget> {
        TODO("Not yet implemented")
    }

    override fun update(deltaTime: Double): PointInput {
        TODO("Not yet implemented")
    }

}