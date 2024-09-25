package sigmacorns.common.control.swerve

import net.unnamedrobotics.lib.control.controller.GeneralController
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Vector2
import kotlin.math.PI
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import kotlin.math.absoluteValue

const val TICKS_PER_REV: Double = 20480.0

//the only state controllers should have is dt
//guard against encoder wraparound in pid controller.
data class ModuleInput(var turn: Double, var drive: Double)
typealias ModuleState = Double // turn encoder ticks
data class ModuleTarget(var v: Vector2, var keepOrientation: Boolean) // direction + speed wheel should be spinning at.

//The job of the ModuleController is to convert target
class ModuleController(
    val turnController: PIDController,
    override var position: ModuleState,
    override var target: ModuleTarget
): GeneralController<ModuleState,ModuleInput,ModuleTarget>() {

    //either -1 or 1
    var reverseEncoders = 1.0
    var drivePowerReversed = 1.0
    var turnPowerReversed = 1.0

    override fun update(dt: Double): ModuleInput {
        var thetaRef = target.v.angleFromOrigin
        val cur = tickToAngle(position.toInt())

        var diff = normalizeRadians(thetaRef - cur)
        val flipped = diff.absoluteValue > PI/2.0
        if(flipped) thetaRef = normalizeRadians(thetaRef - PI)
        diff = normalizeRadians(thetaRef - cur)

        var driveDir = (if(flipped) -1 else 1)*drivePowerReversed

        if (target.keepOrientation) driveDir = 0.0
        val targetTicks = position + diff/(2*PI) * TICKS_PER_REV

        val turnPow = turnController.updateStateless(dt,position,targetTicks)
        val drivePow = target.v.magnitude*driveDir
        return ModuleInput(turnPow, drivePow)
    }

    override fun copy() = ModuleController(turnController,position,target)
}

fun tickToAngle(pos: Int): Double {
    //-2pi to 2pi
    val a = 2*PI*((pos.toDouble()/ TICKS_PER_REV)%1.0)
    //-pi to pi
    return normalizeRadians(a)
}