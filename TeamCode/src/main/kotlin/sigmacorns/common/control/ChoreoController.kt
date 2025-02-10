package sigmacorns.common.control

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math2.Transform2D
//
//class ChoreoController: Controller<Transform2D, Transform2D, Trajectory<SwerveSample>?>() {
//    override var output: Transform2D = Transform2D(0.m/s, 0.m/s, 0.rad/s)
//    override var position: Transform2D = Transform2D(0.m,0.m,0.rad)
//    override var target: Trajectory<SwerveSample>? = null
//
//    override fun copy(): Controller<Transform2D, Transform2D, Trajectory<SwerveSample>?> {
//        TODO("Not yet implemented")
//    }
//
//    override fun update(deltaTime: Double): Transform2D {
//        if(target==null) return Transform2D(0.m/s, 0.m/s, 0.rad/s)
//        val t = target!!
//
//    }
//}