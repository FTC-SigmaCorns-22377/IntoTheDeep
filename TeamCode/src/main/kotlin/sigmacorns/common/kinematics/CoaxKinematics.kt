package sigmacorns.common.kinematics

import eu.sirotin.kotunil.core.Expression
import net.unnamedrobotics.lib.physics.Kinematics
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math2.unitless

data class CoaxKinematics(
    val ratio1: Expression = 1.0.unitless(),
    val ratio2: Expression = 1.0.unitless()
): Kinematics<DiffyInputPose, DiffyOutputPose> {
    override fun inverse(p: DiffyOutputPose) = DiffyInputPose(
        p.axis1/ratio1,
        (p.axis2-p.axis1)/ratio2
    )

    override fun forward(p: DiffyInputPose) = DiffyOutputPose(
        p.axis1*ratio1,
        p.axis1*ratio1+p.axis2*ratio2
    )
}
