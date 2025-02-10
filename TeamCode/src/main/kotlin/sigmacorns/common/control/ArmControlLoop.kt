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
    servo1: Actuator<Double>,
    servo2: Actuator<Double>,
    override var x: DiffyOutputPose,
    io: SigmaIO
): ControlLoop<DiffyOutputPose, DiffyInputPose, DiffyOutputPose>("arm",io) {
    val kinematics = DiffyKinematics(1.unitless(), (-1).unitless())

    val servoController1 = SimpleServoController(
        servo1,
        Limits.ARM_SERVO_1,
        Tuning.ARM_SERVO_VEL,
        Tuning.ARM_SERVO_TOLERANCE,
        ServoControlLoopState(kinematics.inverse(x).axis1.cast(rad),0.rad/s),io
    )

    val servoController2 = SimpleServoController(
        servo2,
        Limits.ARM_SERVO_2,
        Tuning.ARM_SERVO_VEL,
        Tuning.ARM_SERVO_TOLERANCE,
        ServoControlLoopState(kinematics.inverse(x).axis2.cast(rad),0.rad/s),io
    )

    override var t: DiffyOutputPose = x
    override fun read(): DiffyOutputPose
       = kinematics.forward(DiffyInputPose(
           servoController1.read().estimatedPos,
           servoController2.read().estimatedPos
       ))

    override fun reached(x: DiffyOutputPose, t: DiffyOutputPose)
        = servoController1.reached() && servoController2.reached()

    override fun write(u: DiffyInputPose) {
        servoController1.write(u.axis1.value)
        servoController2.write(u.axis2.value)
    }

    override var u: DiffyInputPose = update(0.0)

    override fun update(deltaTime: Double): DiffyInputPose {
        val servoTargets = kinematics.inverse(t)

        servoController1.t = servoTargets.axis1.cast(rad)
        servoController2.t = servoTargets.axis2.cast(rad)

        x = kinematics.forward(DiffyInputPose(servoController1.x.estimatedPos,servoController2.x.estimatedPos))

        return DiffyInputPose(
            servoController1.update(deltaTime).unitless(),
            servoController2.update(deltaTime).unitless()
        )
    }
}