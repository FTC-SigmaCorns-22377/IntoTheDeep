package sigmacorns.common.control

import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.cast
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.IntakeAngleKinematics
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning

class IntakeController(
    servo1: Actuator<Double>,
    servo2: Actuator<Double>,
    override var x: Radian,
    io: SigmaIO,
): ControlLoop<Radian, Pair<Double,Double>, Radian>("intake",io) {
    val kinematics = IntakeAngleKinematics

    val servoController1 = ServoController(
        servo1,
        Limits.INTAKE_SERVO_1,
        Tuning.INTAKE_SERVO_ACC,
        Tuning.INTAKE_SERVO_VEL,
        Tuning.INTAKE_SERVO_TOLERANCE,
        ServoControlLoopState(kinematics.inverse(x),0.rad/s),
        io
    )

    val servoController2 = ServoController(
        servo2,
        Limits.INTAKE_SERVO_2,
        Tuning.INTAKE_SERVO_ACC,
        Tuning.INTAKE_SERVO_VEL,
        Tuning.INTAKE_SERVO_TOLERANCE,
        ServoControlLoopState(kinematics.inverse(x),0.rad/s),
        io
    )

    override fun update(deltaTime: Double): Pair<Double,Double> {
        val servoAngle = IntakeAngleKinematics.inverse(t)
        servoController1.t = servoAngle
        servoController2.t = servoAngle

        return servoController1.update(deltaTime) to servoController2.update(deltaTime)
    }

    override var t: Radian = x
    override var u = update(0.0)

    override fun read(): Radian = servoController1.x.estimatedPos.cast(rad)

    override fun reached(x: Radian, t: Radian): Boolean
        = servoController1.reached() && servoController2.reached()

    override fun write(u: Pair<Double,Double>) {
        servoController1.write(u.first)
        servoController2.write(u.second)
    }
}