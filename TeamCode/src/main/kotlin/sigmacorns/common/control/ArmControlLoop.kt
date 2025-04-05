package sigmacorns.common.control

import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.rad
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.unitless
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyKinematics
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning

class ArmControlLoop(
    val servo1: (Double) -> Unit,
    val servo2: (Double) -> Unit,
    override var x: DiffyOutputPose,
    io: SigmaIO
): ControlLoop<DiffyOutputPose, DiffyInputPose, DiffyOutputPose>("arm",io) {
    val kinematics = DiffyKinematics(1.unitless(), (-1).unitless())

    val servoController1 = SimpleServoController(
        Limits.ARM_SERVO_1,
        Tuning.ARM_SERVO_VEL,
        Tuning.ARM_SERVO_TOLERANCE,
        ServoControlLoopState(kinematics.inverse(x).axis1.cast(rad),0.rad/s)
    )

    val servoController2 = SimpleServoController(
        Limits.ARM_SERVO_2,
        Tuning.ARM_SERVO_VEL,
        Tuning.ARM_SERVO_TOLERANCE,
        ServoControlLoopState(kinematics.inverse(x).axis2.cast(rad),0.rad/s)
    )

    override var t: DiffyOutputPose = x
    override fun read(): DiffyOutputPose
       = kinematics.forward(DiffyInputPose(
           servoController1.position.estimatedPos,
           servoController2.position.estimatedPos
       ))

    override fun reached(x: DiffyOutputPose, t: DiffyOutputPose)
        = servoController1.reached() && servoController2.reached()

    override fun write(u: DiffyInputPose) {
        (servo1)(u.axis1.value)
        (servo2)(u.axis2.value)
    }

    override var u: DiffyInputPose = update(0.0)

    override fun update(deltaTime: Double): DiffyInputPose {
        val servoTargets = kinematics.inverse(t)

        servoController1.target = servoTargets.axis1.cast(rad)
        servoController2.target = servoTargets.axis2.cast(rad)

        x = kinematics.forward(DiffyInputPose(servoController1.position.estimatedPos,servoController2.position.estimatedPos))

        return DiffyInputPose(
            servoController1.update(deltaTime).unitless(),
            servoController2.update(deltaTime).unitless()
        )
    }
}