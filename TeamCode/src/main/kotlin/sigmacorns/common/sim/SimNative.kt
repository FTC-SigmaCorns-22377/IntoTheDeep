package sigmacorns.common.sim

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.clampMagnitude
import java.io.Closeable

data class SimInput(
    //drives
    val FL: Double,
    val BL: Double,
    val BR: Double,
    val FR: Double,

    // extend motors
    val motor1: Double,
    val motor2: Double,
) {
    fun serialize(): DoubleArray
        = listOf(
            FL,
            BL,
            BR,
            FR,

            // extend motors
            motor1,
            motor2,
        ).map { fixMotorPowers(it) }.toDoubleArray()

    private fun fixMotorPowers(pow: Double)
    = pow.clampMagnitude(1.0).takeUnless { it.isNaN() } ?: 0.0
}


data class SimOutput(
    val motor1: Double,
    val motor2: Double,
    val pos: Transform2D,
    val vel: Twist2D
) {
    companion object {
        fun deserialize(data: DoubleArray): SimOutput {
            return SimOutput(
                data[6],
                data[7],
                Transform2D(data[0].m,data[1].m,data[2].rad),
                Twist2D(data[3].m/s,data[4].m/s, data[5].rad/s)
            )
        }
    }
}

class SimNative(
    motor1: Double,
    motor2: Double,
    pose: Transform2D,
): Closeable {
    init {
         System.loadLibrary("ftcrobotcontroller")
    }

    private external fun step(dt: Double, input: DoubleArray, ptr: Long): DoubleArray
    private external fun initial(motorPos1: Double, motorPos2: Double, x: Double, y: Double, theta: Double): Long
    private external fun free(ptr: Long)

    private val ptr = initial(motor1, motor2, pose.x.value, pose.y.value, pose.angle.value);
    override fun close() = free(ptr);

    var output: SimOutput = SimOutput(motor1, motor2, pose, Twist2D(0.m/s,0.m/s, 0.rad/s))

    fun step(dt: Double, input: SimInput) {
        output = SimOutput.deserialize(step(dt, input.serialize(), ptr))
    }
}