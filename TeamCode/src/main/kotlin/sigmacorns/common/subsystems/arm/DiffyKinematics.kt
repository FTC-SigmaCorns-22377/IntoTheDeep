package sigmacorns.common.subsystems.arm

import eu.sirotin.kotunil.core.Expression
import net.unnamedrobotics.lib.physics.Kinematics
import eu.sirotin.kotunil.core.*

data class DiffyOutputPose(var axis1: Expression, var axis2: Expression)
data class DiffyInputPose(var axis1: Expression, var axis2: Expression)
data class DiffyKinematics(
    val ratio1: Expression,
    val ratio2: Expression
): Kinematics<DiffyInputPose,DiffyOutputPose> {
    override fun inverse(p: DiffyOutputPose) = DiffyInputPose(
        (p.axis1/ratio1 + p.axis2/ratio2),
        (p.axis1/ratio1 - p.axis2/ratio2)
    )

    override fun forward(p: DiffyInputPose) = DiffyOutputPose(
        ((p.axis1+p.axis2)/2.0*ratio1),
        ((p.axis1-p.axis2)/2.0*ratio2)
    )
}
