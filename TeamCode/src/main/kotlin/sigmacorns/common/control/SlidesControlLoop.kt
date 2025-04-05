package sigmacorns.common.control

import eu.sirotin.kotunil.base.m
import sigmacorns.common.io.SigmaIO
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.checkedUnitless
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning
import kotlin.math.absoluteValue

fun slidesControlLoop(
    controller: PIDDiffyController,
    io: SigmaIO,
    init: DiffyOutputPose,
): ControlLoop<DiffyInputPose, List<Volt>, DiffyOutputPose> {
    return controlLoop(controller, "slides", io,
        { DiffyInputPose(io.motor1Pos(), io.motor2Pos()) },
        { u ->
            io.motor1 = (u[0] / io.voltage()).checkedUnitless()
            io.motor2 = (u[1] / io.voltage()).checkedUnitless()
        },
        { x, t ->
            val cur = controller.kinematics.forward(x)
            val tBounded = DiffyOutputPose(
                Limits.EXTENSION.apply(t.axis1.cast(m)),
                Limits.LIFT.apply(t.axis2.cast(m))
            )

            val extensionError = (tBounded.axis1 - cur.axis1)
            val liftError = (tBounded.axis2 - cur.axis2)

            val extensionReached = extensionError.map { it.absoluteValue } < Tuning.EXTENSION_TOLERANCE;
            val liftReached = liftError.map { it.absoluteValue } < Tuning.LIFT_TOLERANCE

            extensionReached && liftReached
        }
    ).also { it.t = init }
}