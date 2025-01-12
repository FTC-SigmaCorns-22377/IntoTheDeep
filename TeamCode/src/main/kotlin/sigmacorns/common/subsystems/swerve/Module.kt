package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.cos
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import kotlin.math.PI
import kotlin.math.absoluteValue

const val TICKS_PER_REV: Double = 20480.0

/**
 * @param turn the power to supply to the turn servo. -1.0 ≤ turn ≤ 1.0
 * @param drive the power to supply to the drive motor. -1.0 ≤ drive ≤ 1.0*/
data class ModuleInput(var turn: Double, var drive: Double)

/**
 * Current turn encoder ticks
 */
typealias ModuleState = Radian

/**
 * @param v the target robot-relative velocity of the module.
 * @param powerDriveMotors whether or not to power the drive motors. This is used whenever we want to point the modules in a direction without powering them, like for locking wheels.
 *  */
data class ModuleTarget(var angle: Radian, var drivePower: Double)


/**
 * The job of the ModuleController is to convert a [ModuleTarget] (velocity vector)
 * into a [ModuleInput] (drive + turn powers) given the current [ModuleState] (module rotation)
 * @param turnController PID controller for the turn servo. This converts target servo ticks into the power to supply the servo.
 * @param position initial position
 * @param target initial target
 */

class ModuleController(
    val turnCoefficients: PIDCoefficients,
): Controller<ModuleState, ModuleInput, ModuleTarget>() {
    val turnController = PIDController(turnCoefficients)

    override lateinit var position: ModuleState
    override lateinit var target: ModuleTarget
    override var output: ModuleInput = ModuleInput(0.0,0.0)

    override fun update(dt: Double): ModuleInput {
        //fetch current and desired position in radians
        var thetaRef = target.angle

        //find shortest distance to turn, flipping target and motor powers if > 90°
        var diff = (thetaRef - position).normalizeRadian()
        val flipped = diff.map { it.absoluteValue } > 90.degrees

        if(flipped) thetaRef = (thetaRef - 180.degrees).normalizeRadian()
        diff = (thetaRef - position).normalizeRadian()

        //drive direction ∈ [-1,0,1]
        val driveDir = if(flipped) -1.0 else 1.0

        //convert radians back to ticks
        val turnTarget = position + diff/(2*PI)

        //update turn controller with target
        val turnPow = turnController.updateStateless(dt,position.value,turnTarget.value)

        val drivePow = target.drivePower.clampMagnitude(1.0) * driveDir * diff.cos()

        return ModuleInput(turnPow, drivePow)
    }

    override fun copy() = ModuleController(turnCoefficients)
}