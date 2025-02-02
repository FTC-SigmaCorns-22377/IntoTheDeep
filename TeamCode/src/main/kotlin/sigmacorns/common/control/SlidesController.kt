package sigmacorns.common.control

import sigmacorns.common.io.SigmaIO
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.checkedUnitless
import net.unnamedrobotics.lib.math2.map
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyKinematics
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Physical
import sigmacorns.constants.Tuning
import kotlin.math.absoluteValue

fun slidesControlLoop(io: SigmaIO, init: DiffyOutputPose): ControlLoop<DiffyInputPose, List<Volt>, DiffyOutputPose> {
    val kinematics = DiffyKinematics(Physical.EXTEND_M_PER_TICK, Physical.LIFT_M_PER_TICK)

    val controller = PIDDiffyController(
        kinematics,
        Tuning.EXTENSION_PID,
        Tuning.LIFT_PID,
        Limits.EXTENSION as Bounds<Expression>,
        Limits.LIFT as Bounds<Expression>,
        Limits.EXTENSION_SAFE_THRESH,
        Limits.LIFT_SAFE_THRESH,
        Limits.EXTENSION_SAFE_POWER,
        Limits.LIFT_SAFE_POWER,
        Limits.SLIDE_MOTOR_MAX
    )

    return controller.toControlLoop("slides",io,
        { DiffyInputPose(io.motor1Pos(), io.motor2Pos()) },
        { u ->
            io.motor1 = (u[0]/io.voltage()).checkedUnitless()
            io.motor2 = (u[1]/io.voltage()).checkedUnitless()
        },
        { x, t ->
            val cur = kinematics.forward(x)
            ((t.axis1-cur.axis1).map { it.absoluteValue } < Tuning.EXTENSION_TOLERANCE &&
                (t.axis2-cur.axis2).map { it.absoluteValue } < Tuning.LIFT_TOLERANCE)
        }
    ).also { it.t = init }
}