package sigmacorns.common.swerve

import net.unnamedrobotics.lib.control.controller.GeneralController
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Vector2
import kotlin.math.PI
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import kotlin.math.absoluteValue

const val TICKS_PER_REV: Double = 20480.0

/**
 * @param turn the power to supply to the turn servo. -1.0 ≤ turn ≤ 1.0
 * @param drive the power to supply to the drive motor. -1.0 ≤ drive ≤ 1.0*/
data class ModuleInput(var turn: Double, var drive: Double)

/**
 * Current turn encoder ticks
 */
typealias ModuleState = Double

/**
 * @param v the target robot-relative velocity of the module.
 * @param powerDriveMotors whether or not to power the drive motors. This is used whenever we want to point the modules in a direction without powering them, like for locking wheels.
 *  */
data class ModuleTarget(var v: Vector2, var powerDriveMotors: Boolean)


/**
 * The job of the ModuleController is to convert a [ModuleTarget] (velocity vector)
 * into a [ModuleInput] (drive + turn powers) given the current [ModuleState] (module rotation)
 * @param turnController PID controller for the turn servo. This converts target servo ticks into the power to supply the servo.
 * @param position initial position
 * @param target initial target
 */

class ModuleController(
    val turnController: PIDController,
    override var position: ModuleState = 0.0,
    override var target: ModuleTarget = ModuleTarget(Vector2(),true)
): GeneralController<ModuleState, ModuleInput, ModuleTarget>() {
    override fun update(dt: Double): ModuleInput {
        //fetch current and desired position in radians
        var thetaRef = target.v.angleFromOrigin
        val cur = tickToAngle(position.toInt())

        //find shortest distance to turn, flipping target and motor powers if > 90°
        var diff = normalizeRadians(thetaRef - cur)
        val flipped = diff.absoluteValue > PI/2.0

        if(flipped) thetaRef = normalizeRadians(thetaRef - PI)
        diff = normalizeRadians(thetaRef - cur)

        //drive direction ∈ [-1,0,1]
        val driveDir = if(!target.powerDriveMotors) 0.0 else if(flipped) -1.0 else 1.0

        //convert radians back to ticks
        val targetTicks = position + diff/(2*PI) * TICKS_PER_REV

        //update turn controller with target
        val turnPow = turnController.updateStateless(dt,position,targetTicks)
        val drivePow = target.v.magnitude*driveDir
        return ModuleInput(turnPow, drivePow)
    }

    override fun copy() = ModuleController(turnController,position,target)
}

/**
 * @param pos module turn encoder ticks
 * @return the angle of the module in the range -pi to pi
 * */
fun tickToAngle(pos: Int): Double {
    //-2pi to 2pi
    val a = 2*PI*((pos.toDouble()/ TICKS_PER_REV)%1.0)
    //-pi to pi
    return normalizeRadians(a)
}