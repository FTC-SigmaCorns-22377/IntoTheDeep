package sigmacorns.common.subsystems.scoring

import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.transfer
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.DiffyInputPose
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringTarget
import sigmacorns.common.subsystems.arm.boxTubeKinematics

//fun scoringController() = transfer { dt,x: Transform2D,t:  ->
//
//}
//
//class ScoringController: Controller<ScoringController.State, ScoringController.Input, ScoringController.Target>() {
//    data class State(val pose: Transform2D, val encoderPositions: List<Tick>)
//    data class Target(val pos: Vector3, val pitch: Radian, val roll: Radian)
//    data class Input(val headingTarget: Radian, val armTarget: ArmTarget)
//    override var output: Input = TODO()
//    override var position: State = TODO()
//    override var target: Target = TODO()
//
//    override fun copy(): Controller<State, Input, Target> {
//        TODO("Not yet implemented")
//    }
//
//    override fun update(deltaTime: Double): Target {
//        ScoringKinematics.inverse(ScoringTarget(
//            position.pose.vector(),
//            position.pose.angle.cast(rad),
//            target.pos,
//            target.roll,
//            target.pitch
//        ))
//
//        val armEndPos = boxTubeKinematics.forward(position.encoderPositions.let { DiffyInputPose(it[0],it[1])})
//    }
//}